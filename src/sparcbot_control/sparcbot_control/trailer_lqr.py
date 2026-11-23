import math
import sys
import time
from math import sqrt
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from sparcbot_interfaces.msg import TrailerAngle

# The trailer: measured on the robot.
L = 0.081                               # L: hitch pivot to the centre of the trailer's axle [m]
a = 0.15                                # a: how far BEHIND the robot's axle centre the pivot is [m]

Q = 2.0                                 # cost on the trailer-angle error
R = 1.0                                 # cost on the steering correction

SPEED_LEVELS = (0.10, 0.15, 0.20, -0.10, -0.15, -0.20)       # [m/s]
TURN_LEVELS = (0.10, 0.20, 0.30, -0.10, -0.20, -0.30, 0.0)        # [rad/s]
MAX_STEADY_ANGLE_DEG = 35.0             # arcs with a bigger steady trailer angle are not in the table

RATE_HZ = 50.0                  # control rate
MAX_ANGULAR_RAD_S = 1.0         # limit on the angular.z we publish
MAX_TRAILER_ANGLE_DEG = 60.0    # stop if |phi| is bigger than this (jackknife is near)
TRAILER_TIMEOUT_S = 0.2         # stop if /trailer_angle is silent for this long


def steady_trailer_angle(v, omega):
    # calculating the stable trailer angle for a given linear and angular velocity
    # combination of v and omega gives us a radius which that combination will make 
    # and using that we calculate the angle
    if v == 0.0:
        return None                       # not a circle (stopped or spinning in place)
    k = omega / v                         # curvature [1/m]
    c = a * k
    delta = math.atan(c)
    s = -L * k / math.sqrt(1.0 + c * c)   # value that sin(phi + delta) must equal
    if abs(s) > 1.0:
        return None                       # circle too tight: no steady trailer angle

    return math.asin(s) - delta

def linearize(v, omega):
    #linearizing the trailer angle dynamics at the given v and omega, phi is the trailer angle wrt to sparcbot
    #the dynamics for trailer angle: phi_dot = -v/L * sin(phi) - (1 + a/L * cos(phi)) * omega
    #you will be returning phi, A, B
    #TODO
    raise NotImplementedError('linearize: fill in the TODO')

def scalar_lqr(A, B, Q, R):
    # using the ricatti equation to calcualate P the root using this quadratic equation where P > 0
    # 2 * A * P - (B^2 / R) * P^2 + Q = 0 
    # and the gain K is calculated using K = B * P/R
    # you will be returning K
    #TODO
    raise NotImplementedError('scalar_lqr: fill in the TODO')
    
def build_row(v, omega, Q, R, max_angle_deg=35.0):
    # one row in the table corresponds to a dictionary of the form 
    # {'v': v, 'omega': omega, 'phi_star': phi, 'A': A, 'B': B, 'K': K}
    # we use this build row helper by running in a loop over multiple v and omega combinations
    # to generate a table for gain scheduling
    # you will be returning a dictiopna of this form : {'v': v, 'omega': omega, 'phi_star': phi, 'A': A, 'B': B, 'K': K}
    #TODO
    raise NotImplementedError('build_row: fill in the TODO')

def build_table(Q, R, speed_levels, turn_levels, max_angle_deg=35.0):
    # Collect all those rows built in build row 
    # into a list to make a gains lookup table
    rows = []
    #TODO
    raise NotImplementedError('build_table: fill in the TODO')


def find_row(rows, v_cmd, omega_cmd, tol=1e-3):
    # The table row for this v_cmd and omega_cmd, or None if it is not in the table
    for row in rows:
        if abs(row['v'] - v_cmd) < tol and abs(row['omega'] - omega_cmd) < tol:
            return row
    return None

def scheduled_omega(row, omega_cmd, phi_measured, max_omega):
    # omega_out = omega_cmd - K * (phi - phi_star), clamped to +/- max_omega."""
    omega_out = omega_cmd - row['K'] * (phi_measured - row['phi_star'])
    return max(-max_omega, min(max_omega, omega_out))


def run_tests():
    # Self-test of the helper functions (no robot, no ROS needed to run it):
    #   ros2 run sparcbot_control trailer_lqr test
    # A helper you have not written yet shows as TODO, not FAIL. The helpers are tested in the
    # order you write them: linearize, scalar_lqr, build_row, build_table.
    # The reference values are for L=0.081, a=0.14, Q=R=1 and a 35 deg limit, whatever the
    # settings at the top of this file say, so they only change if the maths changes.
    global L, a
    settings_L, settings_a = L, a       # the file's settings, put back before the last section
    L, a = 0.081, 0.14                  # the reference values below are for these
    failures = []
    waiting = []                        # helpers that still raise NotImplementedError
    NOT_DONE = object()
    skipped = {'checks': 0, 'missing': set()}     # checks of the current section waiting on a TODO

    def attempt(fn, *args):
        # fn(*args), or NOT_DONE if it (or a helper it calls) is not written yet
        try:
            return fn(*args)
        except NotImplementedError as e:
            name = str(e).split(':')[0] or fn.__name__
            if name not in waiting:
                waiting.append(name)
            skipped['missing'].add(name)
            return NOT_DONE

    def heading(title):
        end_section()
        print(title)

    def end_section():
        if skipped['checks']:
            names = ', '.join(sorted(skipped['missing']))
            print(f"TODO  {skipped['checks']} checks skipped until {names} is written")
        skipped['checks'] = 0
        skipped['missing'] = set()

    def check(name, got, expected, tol=5e-4):
        if got is NOT_DONE:
            skipped['checks'] += 1
            return
        ok = got is not None and abs(got - expected) <= tol
        print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got if got is None else round(got, 4)}, "
            f"expected {expected}")
        if not ok:
            failures.append(name)

    def check_true(name, condition):
        if condition is NOT_DONE:
            skipped['checks'] += 1
            return
        print(f"{'PASS' if condition else 'FAIL'}  {name}")
        if not condition:
            failures.append(name)

    def field(result, key, convert=None):
        # one entry of a result (a tuple or a row), passing NOT_DONE and None through
        if result is NOT_DONE or result is None:
            return result
        return result[key] if convert is None else convert(result[key])

    def todo_if(result, condition):
        return NOT_DONE if result is NOT_DONE else condition

    reference_points = [
        # v, omega, phi* [deg], A, B
        (-0.20, 0.0, 0.0, 2.4691, -2.7284),
        (-0.20, 0.3, 18.69, 2.5051, -2.6373),
        (-0.15, 0.2, 16.67, 1.8732, -2.6558),
        (-0.10, -0.1, -12.57, 1.2426, -2.6870),
        (0.10, 0.0, 0.0, -1.2346, -2.7284),
        (0.20, -0.3, 18.69, -2.5051, -2.6373)]
    reference_gains = [-2.2537, -2.3291, -1.9290, -1.5642, -0.6451, -0.4293]

    heading('steady_trailer_angle [deg]')
    for v, w, expected in [(0.2, 0.0, 0.0), (-0.2, 0.3, 18.69), (-0.1, -0.1, -12.57),
                        (0.2, -0.3, 18.69), (0.1, 0.3, -35.72)]:
        phi = steady_trailer_angle(v, w)
        check(f'v={v:+.2f} w={w:+.2f}', None if phi is None else math.degrees(phi), expected, 0.01)
    check_true('v = 0 has no circle', steady_trailer_angle(0.0, 0.3) is None)
    L, a = 0.30, 0.10
    check_true('too tight a circle has no angle (L=0.30, a=0.10, v=0.1, w=1.0)',
            steady_trailer_angle(0.1, 1.0) is None)
    L, a = 0.081, 0.14

    heading('\nlinearize: phi*, A, B')
    for v, w, phi_deg, A, B in reference_points:
        result = attempt(linearize, v, w)
        tag = f'v={v:+.2f} w={w:+.2f}'
        check(tag + ' phi*', field(result, 0, math.degrees), phi_deg, 0.01)
        check(tag + ' A', field(result, 1), A)
        check(tag + ' B', field(result, 2), B)
    result = attempt(linearize, 0.10, 0.30)
    check_true('linearize does not apply the angle limit ((0.10, 0.30) still gives a result)',
            todo_if(result, result is not None))
    L, a = 0.30, 0.10
    result = attempt(linearize, 0.1, 1.0)
    check_true('linearize returns None when there is no steady angle',
            todo_if(result, result is None))
    L, a = 0.081, 0.14

    heading('\nscalar_lqr: K')
    for A, B, q, r, expected in [(2 / 3, -4 / 3, 1.0, 1.0, -1.6180), (-2 / 3, -4 / 3, 1.0, 1.0, -0.6180),
                                 (2 / 3, -4 / 3, 2.0, 1.0, -2.0), (2 / 3, -4 / 3, 1.0, 2.0, -1.3660)]:
        check(f'A={A:+.4f} B={B:+.4f} Q={q:g} R={r:g}', attempt(scalar_lqr, A, B, q, r), expected)

    heading('\nbuild_row: phi*, A, B, K')
    for (v, w, phi_deg, A, B), K in zip(reference_points, reference_gains):
        row = attempt(build_row, v, w, 1.0, 1.0)
        tag = f'v={v:+.2f} w={w:+.2f}'
        check(tag + ' phi*', field(row, 'phi_star', math.degrees), phi_deg, 0.01)
        check(tag + ' A', field(row, 'A'), A)
        check(tag + ' B', field(row, 'B'), B)
        check(tag + ' K', field(row, 'K'), K)
    row = attempt(build_row, -0.20, 0.0, 1.0, 1.0)
    check_true('the row has the keys v, omega, phi_star, A, B, K',
            NOT_DONE if row is NOT_DONE else
            (row is not None and set(row) == {'v', 'omega', 'phi_star', 'A', 'B', 'K'}))

    heading('\nangle limit (build_row)')
    row = attempt(build_row, 0.10, 0.30, 1.0, 1.0, 35.0)
    check_true('(0.10, 0.30) is refused at 35 deg', todo_if(row, row is None))
    row = attempt(build_row, 0.10, 0.30, 1.0, 1.0, 40.0)
    check_true('(0.10, 0.30) is allowed at 40 deg', todo_if(row, row is not None))

    heading('\nbuild_table (Q = R = 1)')
    rows = attempt(build_table, 1.0, 1.0, (0.10, 0.15, 0.20, -0.10, -0.15, -0.20),
                (0.10, 0.20, 0.30, -0.10, -0.20, -0.30, 0.0))
    if rows is NOT_DONE:
        skipped['checks'] += 4          # row count, rows per speed, stability, mirror symmetry
    elif rows is None:
        print('FAIL  build_table returned None (is there a return rows at the end?)')
        failures.append('build_table returned None')
    else:
        check('number of rows', len(rows), 38, 0)
        for v, n in [(-0.20, 7), (-0.15, 7), (-0.10, 5), (0.10, 5), (0.15, 7), (0.20, 7)]:
            check(f'rows with v={v:+.2f}', sum(1 for r in rows if r['v'] == v), n, 0)
        check_true('every row is stable (A - B*K < 0)', all(r['A'] - r['B'] * r['K'] < 0 for r in rows))
        mirror = True
        for r in rows:
            m = find_row(rows, r['v'], -r['omega'])
            if m is None or abs(m['K'] - r['K']) > 1e-9 or abs(m['phi_star'] + r['phi_star']) > 1e-9:
                mirror = False
        check_true('(v, omega) and (v, -omega) have the same K and opposite phi*', mirror)

    heading('\nfind_row (on a small table written by hand)')
    sample = [{'v': 0.15, 'omega': 0.20}, {'v': -0.20, 'omega': 0.0}]
    check_true('float noise still finds the row (0.15000000000000002, 0.1 + 0.1)',
            find_row(sample, 0.15000000000000002, 0.1 + 0.1) is sample[0])
    check_true('a refused arc is not found', find_row(sample, 0.10, 0.30) is None)
    check_true('stopped is not found', find_row(sample, 0.0, 0.0) is None)
    check_true('an off-grid speed is not found', find_row(sample, 0.12, 0.0) is None)

    heading('\nscheduled_omega (omega_out = omega_cmd - K * (phi - phi*), on rows written by hand)')
    by_hand = {
        (-0.2, 0.0): {'phi_star': 0.0, 'K': -2.2537},
        (-0.2, 0.3): {'phi_star': 0.3262, 'K': -2.3291},
        (-0.1, -0.1): {'phi_star': -0.2194, 'K': -1.5642},
        (0.2, 0.0): {'phi_star': 0.0, 'K': -0.4437},
        (0.2, -0.3): {'phi_star': 0.3262, 'K': -0.4293}}
    for (v, w), phi, expected in [((-0.2, 0.0), 0.10, 0.2254), ((-0.2, 0.3), 0.40, 0.4719),
                                  ((-0.1, -0.1), -0.30, -0.2261), ((0.2, 0.0), 0.10, 0.0444),
                                  ((0.2, -0.3), 0.0, -0.4400)]:
        check(f'v={v:+.2f} w={w:+.2f} phi={phi:+.2f}',
            scheduled_omega(by_hand[(v, w)], w, phi, 1.5), expected)
    check('clamped to the limit (phi = 1.0 would give 2.25)',
        scheduled_omega(by_hand[(-0.2, 0.0)], 0.0, 1.0, 1.5), 1.5)

    heading('\nthe settings at the top of this file')
    L, a = settings_L, settings_a
    live = attempt(build_table, Q, R, SPEED_LEVELS, TURN_LEVELS, MAX_STEADY_ANGLE_DEG)
    if live is NOT_DONE:
        skipped['checks'] += 3          # not empty, every row stable, closed loop
    elif live is None:
        print('FAIL  build_table returned None (is there a return rows at the end?)')
        failures.append('build_table returned None')
    else:
        check_true('the table is not empty', len(live) > 0)
        check_true('every row is stable (A - B*K < 0)', all(r['A'] - r['B'] * r['K'] < 0 for r in live))
        check_true('closed loop A - B*K = -sqrt(A^2 + B^2*Q/R) in every row',
                all(abs((r['A'] - r['B'] * r['K']) + sqrt(r['A'] ** 2 + r['B'] ** 2 * Q / R)) < 1e-9
                    for r in live))
        row = find_row(live, -max(SPEED_LEVELS), 0.0)
        print(f'info  {len(live)} rows with Q={Q:g}, R={R:g}; 'f'K(v={-max(SPEED_LEVELS):+.2f}, w=0) = {row["K"]:.4f}' if row else f'info  {len(live)} rows')

    end_section()
    print()
    if failures:
        print(f'{len(failures)} FAILED: ' + ', '.join(failures))
    if waiting:
        print('Still to write (shown as TODO above): ' + ', '.join(waiting))
    if failures or waiting:
        return 1
    print('all checks passed')
    return 0


def main(args=None):
    # `trailer_lqr test` checks the maths and exits; with no argument the node runs.
    if 'test' in (sys.argv[1:] if args is None else args):
        sys.exit(run_tests())

    # Keep the ROS context alive on Ctrl-C so the final zero /cmd_vel can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('trailer_lqr')

    # Built once here; takes well under a millisecond.
    rows = build_table(Q, R, SPEED_LEVELS, TURN_LEVELS, MAX_STEADY_ANGLE_DEG)
    if not rows:
        raise RuntimeError('build_table gave no rows (does it end with `return rows`?)')

    # What the callbacks share.
    state = {
        'v': 0.0, 'w': 0.0,             # the held teleop command
        'trailer': None,                # latest TrailerAngle
        'trailer_time': None,           # when it arrived
        'sending': False,               # True while we are publishing a motion
    }

    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    debug_pub = node.create_publisher(Vector3Stamped, '~/debug', 10)

    def publish(v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        cmd_pub.publish(cmd)

    def stop(reason):
        """Zero velocity, and forget the held command: press a key again to restart."""
        state['v'] = 0.0
        state['w'] = 0.0
        publish(0.0, 0.0)
        node.get_logger().warn(f'STOPPED: {reason} Press a key to drive again.')
        state['sending'] = False

    def on_teleop(msg):
        state['v'] = msg.linear.x
        state['w'] = msg.angular.z

    def on_trailer(msg):
        state['trailer'] = msg
        state['trailer_time'] = node.get_clock().now()

    def trailer_angle():
        """The trailer angle [rad], or None (after stopping) if it cannot be trusted."""
        trailer = state['trailer']
        if trailer is None:
            stop('no /trailer_angle yet (is trailer_sensor running?).')
            return None

        age_s = (node.get_clock().now() - state['trailer_time']).nanoseconds * 1e-9
        if age_s > TRAILER_TIMEOUT_S:
            stop(f'no /trailer_angle for {age_s:.2f} s.')
            return None

        if not trailer.magnet_ok:
            stop('the trailer sensor magnet is not OK, so the angle is not reliable.')
            return None

        phi = trailer.angle_rad
        if abs(phi) > math.radians(MAX_TRAILER_ANGLE_DEG):
            stop(f'trailer angle {math.degrees(phi):.1f} deg '
                f'(limit {MAX_TRAILER_ANGLE_DEG:.0f} deg). Jackknife risk.')
            return None
        return phi

    def on_timer():
        v_cmd, w_cmd = state['v'], state['w']

        # Teleop says stop: send ONE zero (so it takes effect), then stay quiet. Staying quiet
        # matters: any /cmd_vel would take the wheels out of the teleop's coast or brake.
        if v_cmd == 0.0 and w_cmd == 0.0:
            if state['sending']:
                publish(0.0, 0.0)
                state['sending'] = False
            return

        phi = trailer_angle()
        if phi is None:
            return

        row = find_row(rows, v_cmd, w_cmd)
        if row is None:
            if v_cmd < 0.0:
                stop('reversing with a command that is not in the gain table '
                    f'(v={v_cmd:+.2f}, w={w_cmd:+.2f}).')
                return
            publish(v_cmd, w_cmd)       # forward: the trailer is stable on its own
            state['sending'] = True
            return

        w_out = scheduled_omega(row, w_cmd, phi, MAX_ANGULAR_RAD_S)
        publish(v_cmd, w_out)
        state['sending'] = True

        debug = Vector3Stamped()
        debug.header.stamp = node.get_clock().now().to_msg()
        debug.vector.x = float(phi)
        debug.vector.y = float(row['phi_star'])
        debug.vector.z = float(w_out)
        debug_pub.publish(debug)

    node.create_subscription(Twist, 'cmd_vel_teleop', on_teleop, 10)
    trailer_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TrailerAngle, 'trailer_angle', on_trailer, trailer_qos)
    node.create_timer(1.0 / RATE_HZ, on_timer)

    node.get_logger().info(
        f'trailer_lqr ready: {len(rows)} gain rows, '
        f'L={L:.3f} m, a={a:.3f} m, '
        f'Q={Q:g}, R={R:g}. Waiting for /cmd_vel_teleop.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if state['sending']:
            for _ in range(3):
                publish(0.0, 0.0)
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()