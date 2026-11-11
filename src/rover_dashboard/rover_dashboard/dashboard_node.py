"""Rover web dashboard (runs on the Pi). Open http://<pi-ip>:8080 from any browser on the LAN.

Serves a single-page app plus a small JSON API, using only the Python standard library
for HTTP. ROS runs in a background thread.

API
  GET  /api/state                         live status (polled by the page)
  POST /api/cmd            {"c": "w"}     drive command -> rover/cmd (page re-sends while held)
  GET  /api/maps                          stored maps in maps_dir
  GET  /api/maps/<m>/image                map as PNG
  GET  /api/maps/<m>/routes               saved routes for a map
  POST /api/maps/<m>/routes {"name", "route"}   save / overwrite a route
  DELETE /api/maps/<m>/routes/<r>
  POST /api/maps/<m>/go    {"route"|"name", "set_initial_pose", "load_map"}
  POST /api/nav/cancel
  POST /api/maps/save      {"name"}       save the live /map with nav2 map_saver_cli
  GET  /api/live/image                    live SLAM /map as PNG (meta is in /api/state)
  POST /api/explorer/start {"name"}       start autonomous mapping (optional map name)
  POST /api/explorer/stop
  POST /api/localize                      run global localization now
"""
import io
import json
import math
import os
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from PIL import Image
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int8MultiArray, Int16MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')
VALID_CMDS = {'w', 's', 'a', 'd', 'x'}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Dashboard(Node):
    def __init__(self):
        super().__init__('rover_dashboard')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.host = p('host', '0.0.0.0')
        self.port = p('port', 8080)
        self.maps_dir = os.path.expanduser(p('maps_dir', '~/maps'))
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        os.makedirs(self.maps_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.t0 = time.time()
        self.s = {
            'tof': {'distance': None, 'cls': None, 'obstacle': None, 'cliff': None,
                    'nearest': None, 'stamp': 0.0},
            'rover': {'sent': None, 'sent_stamp': 0.0, 'rx': []},
            'pose': None,
            'nav': {'state': 'idle', 'detail': '', 'route': None, 'map': None},
            'nodes': [],
            'explorer': None,          # last /explorer/state JSON
            'loc_score': None,
            'live_map': None,          # meta of the latest /map
            'explorer_path': [],
            'explorer_boundary': [],
        }
        self.live_msg = None
        self._live_png = (None, None)
        self.rx = deque(maxlen=12)

        tof = self.s['tof']

        def tof_set(key, conv=lambda m: m.data):
            def cb(m):
                with self.lock:
                    tof[key] = conv(m)
                    tof['stamp'] = time.time()
            return cb

        self.create_subscription(Int16MultiArray, 'tof/distance_mm',
                                 tof_set('distance', lambda m: list(m.data)), 10)
        self.create_subscription(Int8MultiArray, 'tof/zone_class',
                                 tof_set('cls', lambda m: list(m.data)), 10)
        self.create_subscription(Bool, 'tof/obstacle_detected', tof_set('obstacle'), 10)
        self.create_subscription(Bool, 'tof/cliff_detected', tof_set('cliff'), 10)
        self.create_subscription(
            Float32, 'tof/nearest_obstacle',
            tof_set('nearest', lambda m: None if math.isinf(m.data) else round(m.data, 3)), 10)
        self.create_subscription(String, 'rover/sent', self._on_sent, 10)
        self.create_subscription(String, 'rover/esp32_rx', self._on_rx, 10)
        self.create_subscription(String, 'explorer/state', self._on_explorer, 10)
        self.create_subscription(Float32, 'localization/score', self._on_score, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        self.create_subscription(Path, 'explorer/path', lambda m: self._on_path('explorer_path', m),
                                 map_qos)
        self.create_subscription(Path, 'explorer/boundary',
                                 lambda m: self._on_path('explorer_boundary', m), map_qos)
        self.cli_explore_start = self.create_client(Trigger, 'explorer/start')
        self.cli_explore_stop = self.create_client(Trigger, 'explorer/stop')
        self.cli_localize = self.create_client(Trigger, 'localization/global_localize')
        self.cli_explorer_params = self.create_client(SetParameters, 'explorer/set_parameters')

        self.pub_cmd = self.create_publisher(String, 'rover/cmd', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_route = self.create_publisher(Path, 'rover/route', latched)

        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.load_map_client = self.create_client(LoadMap, 'map_server/load_map')
        self.goal_handle = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self._update_pose)
        self.create_timer(2.0, self._update_nodes)
        self._png_cache = {}

    # ------------------------------------------------------------------ ROS callbacks
    def _on_sent(self, m):
        with self.lock:
            self.s['rover']['sent'] = m.data
            self.s['rover']['sent_stamp'] = time.time()

    def _on_rx(self, m):
        with self.lock:
            self.rx.append([time.strftime('%H:%M:%S'), m.data])

    def _on_explorer(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        st['stamp'] = time.time()
        with self.lock:
            self.s['explorer'] = st

    def _on_score(self, m):
        with self.lock:
            self.s['loc_score'] = round(float(m.data), 3)

    def _on_map(self, m):
        o = m.info.origin.position
        with self.lock:
            self.live_msg = m
            self.s['live_map'] = {'width': m.info.width, 'height': m.info.height,
                                  'resolution': m.info.resolution, 'origin': [o.x, o.y, 0.0],
                                  'stamp': time.time()}

    def _on_path(self, key, m):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in m.poses]
        if len(pts) > 600:
            pts = pts[::len(pts) // 600 + 1] + [pts[-1]]
        with self.lock:
            self.s[key] = pts

    def live_png(self):
        with self.lock:
            m = self.live_msg
        if m is None:
            raise FileNotFoundError('no live /map yet (is slam_toolbox running?)')
        if self._live_png[0] is m:
            return self._live_png[1]
        d = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        img = np.full(d.shape, 205, np.uint8)                     # unknown = grey
        known = d >= 0
        img[known] = (254 - np.clip(d[known], 0, 100) * 2.54).astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(np.flipud(img)).save(buf, 'PNG')
        self._live_png = (m, buf.getvalue())
        return self._live_png[1]

    def _call(self, client, req, what, timeout=5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'{what} is not running')
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        if not fut.done():
            raise RuntimeError(f'{what} did not answer')
        return fut.result()

    def explorer_start(self, name=''):
        if name:
            if not NAME_RE.match(name):
                raise ValueError('map name: letters, numbers, _ and - only')
            req = SetParameters.Request()
            req.parameters = [Parameter(name='map_name', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=name))]
            self._call(self.cli_explorer_params, req, 'explorer')
        r = self._call(self.cli_explore_start, Trigger.Request(), 'explorer')
        return r.message

    def explorer_stop(self):
        r = self._call(self.cli_explore_stop, Trigger.Request(), 'explorer')
        self.drive('x')
        return r.message

    def localize(self):
        r = self._call(self.cli_localize, Trigger.Request(), 'global_localizer', timeout=30.0)
        if not r.success:
            raise RuntimeError(r.message)
        return r.message

    def exploring(self):
        with self.lock:
            st = self.s['explorer']
        return bool(st and time.time() - st['stamp'] < 2.0
                    and st.get('state') not in ('IDLE', 'DONE', 'STOPPED'))

    def _update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            with self.lock:
                self.s['pose'] = None
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.s['pose'] = {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                              'yaw': yaw}

    def _update_nodes(self):
        names = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in self.get_node_names_and_namespaces())
        with self.lock:
            self.s['nodes'] = names

    # ------------------------------------------------------------------ state + commands
    def state(self):
        with self.lock:
            s = json.loads(json.dumps(self.s))
            s['rover']['rx'] = list(self.rx)
        now = time.time()
        s['uptime'] = round(now - self.t0)
        s['tof']['age'] = round(now - s['tof']['stamp'], 2) if s['tof']['stamp'] else None
        s['bridge_running'] = self.count_subscribers('rover/cmd') > 0
        s['nav2_available'] = self.nav_client.server_is_ready()
        return s

    def drive(self, c):
        self.pub_cmd.publish(String(data=c))

    # ------------------------------------------------------------------ maps
    def _map_yaml(self, name):
        if not NAME_RE.match(name):
            raise ValueError('bad map name')
        path = os.path.join(self.maps_dir, name + '.yaml')
        if not os.path.exists(path):
            raise FileNotFoundError(name)
        return path

    def map_info(self, name):
        path = self._map_yaml(name)
        with open(path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(path), img_path)
        with Image.open(img_path) as im:
            w, h = im.size
        origin = meta.get('origin', [0, 0, 0])
        return {'name': name, 'yaml': path, 'image_path': img_path, 'width': w, 'height': h,
                'resolution': float(meta['resolution']),
                'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
                'modified': os.path.getmtime(path)}

    def list_maps(self):
        out = []
        for f in sorted(os.listdir(self.maps_dir)):
            if f.endswith('.yaml') and NAME_RE.match(f[:-5]):
                try:
                    info = self.map_info(f[:-5])
                    info.pop('image_path')
                    info.pop('yaml')
                    info['routes'] = len(self.load_routes(f[:-5]))
                    out.append(info)
                except Exception as e:  # not a map yaml
                    self.get_logger().debug(f'skip {f}: {e}')
        return out

    def map_png(self, name):
        info = self.map_info(name)
        key = (info['image_path'], os.path.getmtime(info['image_path']))
        if key not in self._png_cache:
            with Image.open(info['image_path']) as im:
                buf = io.BytesIO()
                im.convert('L').save(buf, 'PNG')
            self._png_cache = {key: buf.getvalue()}
        return self._png_cache[key]

    def _routes_path(self, name):
        self._map_yaml(name)
        return os.path.join(self.maps_dir, name + '.routes.json')

    def load_routes(self, name):
        path = self._routes_path(name)
        if not os.path.exists(path):
            return {}
        with open(path) as f:
            return json.load(f)

    def save_route(self, name, rname, route):
        if not NAME_RE.match(rname):
            raise ValueError('route name: letters, numbers, _ and - only')
        for key in ('start', 'end'):
            if not route.get(key):
                raise ValueError(f'route needs a {key} point')
        clean = {k: self._clean_pose(route[k]) for k in ('start', 'end')}
        clean['waypoints'] = [self._clean_pose(p) for p in route.get('waypoints', [])]
        clean['saved'] = time.strftime('%Y-%m-%d %H:%M:%S')
        routes = self.load_routes(name)
        routes[rname] = clean
        self._write_routes(name, routes)
        return routes

    def delete_route(self, name, rname):
        routes = self.load_routes(name)
        routes.pop(rname, None)
        self._write_routes(name, routes)
        return routes

    def _write_routes(self, name, routes):
        path = self._routes_path(name)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(routes, f, indent=2)
        os.replace(tmp, path)

    @staticmethod
    def _clean_pose(p):
        return {'x': float(p['x']), 'y': float(p['y']), 'yaw': float(p.get('yaw', 0.0))}

    def save_live_map(self, name):
        if not NAME_RE.match(name):
            raise ValueError('map name: letters, numbers, _ and - only')
        target = os.path.join(self.maps_dir, name)
        r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', target,
                            '--ros-args', '-p', 'save_map_timeout:=5.0'],
                           capture_output=True, text=True, timeout=30)
        if not os.path.exists(target + '.yaml'):
            tail = (r.stdout + r.stderr).strip().splitlines()[-3:]
            raise RuntimeError('map_saver failed (is a /map topic published, e.g. by '
                               'slam_toolbox?) ' + ' | '.join(tail))
        # Also save slam_toolbox's pose graph: needed for localization and continuing the map.
        subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                        'slam_toolbox/srv/SerializePoseGraph', f'{{filename: "{target}"}}'],
                       capture_output=True, text=True, timeout=30)
        if not os.path.exists(target + '.posegraph'):
            return name + ' (image only - slam_toolbox pose graph not saved, so it cannot be ' \
                          'used for localization)'
        return name

    # ------------------------------------------------------------------ navigation
    def _pose_stamped(self, p):
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = p['x'], p['y']
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = yaw_to_quat(p['yaw'])
        return ps

    def go(self, map_name, route, rname, set_initial_pose=True, load_map=True):
        route = {'start': self._clean_pose(route['start']),
                 'end': self._clean_pose(route['end']),
                 'waypoints': [self._clean_pose(p) for p in route.get('waypoints', [])]}
        steps = []
        live = map_name is None   # live SLAM map: slam_toolbox already provides map + localization
        if live:
            set_initial_pose = load_map = False
            steps.append('live SLAM map: skipping map_server load and /initialpose')

        if load_map:
            info = self.map_info(map_name)
            if self.load_map_client.service_is_ready():
                req = LoadMap.Request()
                req.map_url = info['yaml']
                self.load_map_client.call_async(req)
                steps.append('map sent to map_server')
            else:
                steps.append('map_server not running (map not loaded)')

        if set_initial_pose:
            ip = PoseWithCovarianceStamped()
            ps = self._pose_stamped(route['start'])
            ip.header, ip.pose.pose = ps.header, ps.pose
            ip.pose.covariance[0] = ip.pose.covariance[7] = 0.25
            ip.pose.covariance[35] = 0.07
            self.pub_init.publish(ip)
            steps.append('start published to /initialpose')

        poses = [self._pose_stamped(p) for p in route['waypoints'] + [route['end']]]
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self._pose_stamped(route['start'])] + poses
        self.pub_route.publish(path)
        steps.append(f'route published to /rover/route ({len(path.poses)} poses)')

        nav = {'route': rname, 'map': map_name or 'live SLAM map'}
        if self.nav_client.server_is_ready():
            goal = NavigateThroughPoses.Goal()
            goal.poses = poses
            fut = self.nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback)
            fut.add_done_callback(self._nav_accepted)
            nav.update(state='sending', detail='goal sent to Nav2')
            steps.append('goal sent to Nav2 navigate_through_poses')
        else:
            nav.update(state='published', detail='Nav2 not running - route published only')
            steps.append('Nav2 not running: nothing will drive yet')
        with self.lock:
            self.s['nav'] = {**self.s['nav'], **nav}
        return steps

    def _set_nav(self, state, detail):
        with self.lock:
            self.s['nav']['state'], self.s['nav']['detail'] = state, detail

    def _nav_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self._set_nav('rejected', 'Nav2 rejected the goal')
            return
        self.goal_handle = gh
        self._set_nav('active', 'navigating')
        gh.get_result_async().add_done_callback(self._nav_done)

    def _nav_feedback(self, fb):
        f = fb.feedback
        self._set_nav('active', f'{f.distance_remaining:.2f} m remaining, '
                                f'{f.number_of_poses_remaining} poses left')

    def _nav_done(self, fut):
        status = fut.result().status
        names = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}
        self._set_nav(names.get(status, f'finished ({status})'), '')
        self.goal_handle = None

    def cancel(self):
        self.drive('x')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self._set_nav('canceling', '')
            return 'cancel requested'
        self._set_nav('idle', 'stopped')
        return 'no active Nav2 goal; sent stop'


# ====================================================================== HTTP
def make_handler(node, web_dir):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'RoverDashboard/0.1'

        def log_message(self, fmt, *args):  # keep the terminal quiet
            pass

        def _send(self, code, body, ctype='application/json'):
            if not isinstance(body, (bytes, bytearray)):
                body = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            n = int(self.headers.get('Content-Length') or 0)
            return json.loads(self.rfile.read(n) or b'{}') if n else {}

        def _route(self, method):
            parts = [unquote(p) for p in urlparse(self.path).path.strip('/').split('/') if p]
            try:
                if method == 'GET' and parts in ([], ['index.html']):
                    with open(os.path.join(web_dir, 'index.html'), 'rb') as f:
                        return self._send(200, f.read(), 'text/html; charset=utf-8')
                if not parts or parts[0] != 'api':
                    return self._send(404, {'error': 'not found'})
                api = parts[1:]
                if method == 'GET' and api == ['state']:
                    return self._send(200, node.state())
                if method == 'POST' and api == ['cmd']:
                    c = str(self._body().get('c', '')).lower()
                    if c not in VALID_CMDS:
                        return self._send(400, {'error': 'c must be one of w s a d x'})
                    if node.exploring():   # manual input overrides autonomous mapping
                        node.explorer_stop()
                    node.drive(c)
                    return self._send(200, {'ok': True})
                if method == 'GET' and api == ['live', 'image']:
                    return self._send(200, node.live_png(), 'image/png')
                if method == 'POST' and api == ['explorer', 'start']:
                    return self._send(200, {'result': node.explorer_start(
                        str(self._body().get('name', '')).strip())})
                if method == 'POST' and api == ['explorer', 'stop']:
                    return self._send(200, {'result': node.explorer_stop()})
                if method == 'POST' and api == ['localize']:
                    return self._send(200, {'result': node.localize()})
                if method == 'GET' and api == ['maps']:
                    return self._send(200, {'maps': node.list_maps(), 'dir': node.maps_dir})
                if method == 'POST' and api == ['maps', 'save']:
                    return self._send(200, {'saved': node.save_live_map(self._body().get('name', ''))})
                if method == 'POST' and api == ['live', 'go']:
                    b = self._body()
                    if not b.get('route'):
                        return self._send(400, {'error': 'no route'})
                    return self._send(200, {'steps': node.go(None, b['route'], b.get('name') or '(unsaved)')})
                if method == 'POST' and api == ['nav', 'cancel']:
                    return self._send(200, {'result': node.cancel()})
                if len(api) >= 3 and api[0] == 'maps':
                    m = api[1]
                    if method == 'GET' and api[2] == 'image':
                        return self._send(200, node.map_png(m), 'image/png')
                    if method == 'GET' and api[2:] == ['routes']:
                        return self._send(200, {'routes': node.load_routes(m)})
                    if method == 'POST' and api[2:] == ['routes']:
                        b = self._body()
                        return self._send(200, {'routes': node.save_route(m, b.get('name', ''),
                                                                          b.get('route', {}))})
                    if method == 'DELETE' and api[2] == 'routes' and len(api) == 4:
                        return self._send(200, {'routes': node.delete_route(m, api[3])})
                    if method == 'POST' and api[2:] == ['go']:
                        b = self._body()
                        route = b.get('route') or node.load_routes(m).get(b.get('name', ''))
                        if not route:
                            return self._send(400, {'error': 'unknown route'})
                        steps = node.go(m, route, b.get('name') or '(unsaved)',
                                        b.get('set_initial_pose', True), b.get('load_map', True))
                        return self._send(200, {'steps': steps})
                return self._send(404, {'error': 'not found'})
            except FileNotFoundError as e:
                return self._send(404, {'error': f'not found: {e}'})
            except (ValueError, KeyError, TypeError, RuntimeError, json.JSONDecodeError) as e:
                return self._send(400, {'error': str(e)})
            except subprocess.TimeoutExpired:
                return self._send(504, {'error': 'timed out'})

        def do_GET(self):
            self._route('GET')

        def do_POST(self):
            self._route('POST')

        def do_DELETE(self):
            self._route('DELETE')

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    web_dir = os.path.join(get_package_share_directory('rover_dashboard'), 'web')
    httpd = ThreadingHTTPServer((node.host, node.port), make_handler(node, web_dir))
    httpd.daemon_threads = True
    node.get_logger().info(f'Dashboard on http://{node.host}:{node.port}  (maps: {node.maps_dir})')

    def stop_http_when_ros_stops():
        # rclpy handles SIGINT/SIGTERM by shutting down its context; stop the web server then
        # too, otherwise the process would keep serving (and never exit under systemd/launch).
        while rclpy.ok():
            time.sleep(0.3)
        httpd.shutdown()

    threading.Thread(target=stop_http_when_ros_stops, daemon=True).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive('x')
        httpd.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
