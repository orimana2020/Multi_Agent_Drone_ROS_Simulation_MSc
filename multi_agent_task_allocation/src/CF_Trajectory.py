
"""
Example of how to generate trajectories for the High Level commander using
Bezier curves. The output from this script is intended to be pasted into the
autonomous_sequence_high_level.py example.

This code uses Bezier curves of degree 7, that is with 8 control points.
See https://en.wikipedia.org/wiki/B%C3%A9zier_curve

All coordinates are (x, y, z, yaw)
"""
import math
import numpy as np
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
import sys


class Node:
    """
    A node represents the connection point between two Bezier curves
    (called Segments).
    It holds 4 control points for each curve and the positions of the control
    points are set to join the curves with continuity in c0, c1, c2, c3.
    See https://www.cl.cam.ac.uk/teaching/2000/AGraphHCI/SMEG/node3.html

    The control points are named
    p4, p5, p6 and p7 for the tail of the first curve
    q0, q1, q2, q3 for the head of the second curve
    """

    def __init__(self, q0, q1=None, q2=None, q3=None):
        """
        Create a Node. Pass in control points to define the shape of the
        two segments that share the Node. The control points are for the
        second segment, that is the four first control points of the Bezier
        curve after the node. The control points for the Bezier curve before
        the node are calculated from the existing control points.
        The control points are for scale = 1, that is if the Bezier curve
        after the node has scale = 1 it will have exactly these handles. If the
        curve after the node has a different scale the handles will be moved
        accordingly when the Segment is created.

        q0 is required, the other points are optional.
        if q1 is missing it will be set to generate no velocity in q0.
        If q2 is missing it will be set to generate no acceleration in q0.
        If q3 is missing it will be set to generate no jerk in q0.

        If only q0 is set, the node will represent a point where the Crazyflie
        has no velocity. Good for starting and stopping.

        To get a fluid motion between segments, q1 must be set.

        :param q0: The position of the node
        :param q1: The position of the first control point
        :param q2: The position of the second control point
        :param q3: The position of the third control point
        """
        self._control_points = np.zeros([2, 4, 4])

        q0 = np.array(q0)

        if q1 is None:
            q1 = q0
        else:
            q1 = np.array(q1)
            # print('q1 generated:', q1)

        d = q1 - q0

        if q2 is None:
            q2 = q0 + 2 * d
            # print('q2 generated:', q2)
        else:
            q2 = np.array(q2)

        e = 3 * q2 - 2 * q0 - 6 * d

        if q3 is None:
            q3 = e + 3 * d
            # print('q3 generated:', q3)
        else:
            q3 = np.array(q3)

        p7 = q0
        p6 = q1 - 2 * d
        p5 = q2 - 4 * d
        p4 = 2 * e - q3

        self._control_points[0][0] = q0
        self._control_points[0][1] = q1
        self._control_points[0][2] = q2
        self._control_points[0][3] = q3

        self._control_points[1][3] = p7
        self._control_points[1][2] = p6
        self._control_points[1][1] = p5
        self._control_points[1][0] = p4

    def get_head_points(self):
        return self._control_points[0]

    def get_tail_points(self):
        return self._control_points[1]

    def draw_unscaled_controlpoints(self, visualizer):
        color = (0.8, 0.8, 0.8)
        for p in self._control_points[0]:
            visualizer.marker(p[0:3], color=color)
        for p in self._control_points[1]:
            visualizer.marker(p[0:3], color=color)




class Segment:
    """
    A Segment represents a Bezier curve of degree 7. It uses two Nodes to
    define the shape. The scaling of the segment will move the handles compared
    to the Node to maintain continuous position, velocity, acceleration and
    jerk through the Node.
    A Segment can generate a polynomial that is compatible with the High Level
    Commander, either in python to be sent to the Crazyflie, or as C code to be
    used in firmware.
    A Segment can also be rendered in Vispy.
    """

    def __init__(self, head_node, tail_node, scale):
        self._scale = scale

        unscaled_points = np.concatenate(
            [head_node.get_head_points(), tail_node.get_tail_points()])

        self._points = self._scale_control_points(unscaled_points, self._scale)

        polys = self._convert_to_polys()
        self._polys = self._stretch_polys(polys, self._scale)

        self._vel = self._deriv(self._polys)
        self._acc = self._deriv(self._vel)
        self._jerk = self._deriv(self._acc)

    def _convert_to_polys(self):
        n = len(self._points) - 1
        result = np.zeros([4, 8])

        for d in range(4):
            for j in range(n + 1):
                s = 0.0
                for i in range(j + 1):
                    s += ((-1) ** (i + j)) * self._points[i][d] / (
                        math.factorial(i) * math.factorial(j - i))

                c = s * math.factorial(n) / math.factorial(n - j)
                result[d][j] = c

        return result

    def draw_trajectory(self, visualizer):
        self._draw(self._polys, 'black', visualizer)

    def draw_vel(self, visualizer):
        self._draw(self._vel, 'green', visualizer)

    def draw_acc(self, visualizer):
        self._draw(self._acc, 'red', visualizer)

    def draw_jerk(self, visualizer):
        self._draw(self._jerk, 'blue', visualizer)

    def draw_control_points(self, visualizer):
        for p in self._points:
            visualizer.marker(p[0:3])

    def _draw(self, polys, color, visualizer):
        step = self._scale / 32
        prev = None
        for t in np.arange(0.0, self._scale + step, step):
            p = self._eval_xyz(polys, t)

            if prev is not None:
                visualizer.line(p, prev, color=color)

            prev = p

    def velocity(self, t):
        return self._eval_xyz(self._vel, t)

    def acceleration(self, t):
        return self._eval_xyz(self._acc, t)

    def jerk(self, t):
        return self._eval_xyz(self._jerk, t)

    def _deriv(self, p):
        result = []
        for i in range(3):
            result.append([
                1 * p[i][1],
                2 * p[i][2],
                3 * p[i][3],
                4 * p[i][4],
                5 * p[i][5],
                6 * p[i][6],
                7 * p[i][7],
                0
            ])

        return result

    def _eval(self, p, t):
        result = 0
        for part in range(8):
            result += p[part] * (t ** part)
        return result

    def _eval_xyz(self, p, t):
        return np.array(
            [self._eval(p[0], t), self._eval(p[1], t), self._eval(p[2], t)])

    def print_poly_python(self):
        s = '  [' + str(self._scale) + ', '

        for axis in range(4):
            for d in range(8):
                s += str(self._polys[axis][d]) + ', '

        s += '],  # noqa'
        print(s)

    def get_coef(self):
        coef = []
        coef.append(self._scale)
        for axis in range(4):
            for d in range(8):
                coef.append(self._polys[axis][d]) 
        return coef

    def print_poly_c(self):
        s = ''

        for axis in range(4):
            for d in range(8):
                s += str(self._polys[axis][d]) + ', '

        s += str(self._scale)
        print(s)

    def print_points(self):
        print(self._points)

    def print_peak_vals(self):
        peak_v = 0.0
        peak_a = 0.0
        peak_j = 0.0

        step = 0.05
        for t in np.arange(0.0, self._scale + step, step):
            peak_v = max(peak_v, np.linalg.norm(self._eval_xyz(self._vel, t)))
            peak_a = max(peak_a, np.linalg.norm(self._eval_xyz(self._acc, t)))
            peak_j = max(peak_j, np.linalg.norm(self._eval_xyz(self._jerk, t)))

        print('Peak v:', peak_v, 'a:', peak_a, 'j:', peak_j)

    def _stretch_polys(self, polys, time):
        result = np.zeros([4, 8])

        recip = 1.0 / time

        for p in range(4):
            scale = 1.0
            for t in range(8):
                result[p][t] = polys[p][t] * scale
                scale *= recip

        return result

    def _scale_control_points(self, unscaled_points, scale):
        s = scale
        l_s = 1 - s
        p = unscaled_points

        result = [None] * 8

        result[0] = p[0]
        result[1] = l_s * p[0] + s * p[1]
        result[2] = l_s ** 2 * p[0] + 2 * l_s * s * p[1] + s ** 2 * p[2]
        result[3] = l_s ** 3 * p[0] + 3 * l_s ** 2 * s * p[
            1] + 3 * l_s * s ** 2 * p[2] + s ** 3 * p[3]
        result[4] = l_s ** 3 * p[7] + 3 * l_s ** 2 * s * p[
            6] + 3 * l_s * s ** 2 * p[5] + s ** 3 * p[4]
        result[5] = l_s ** 2 * p[7] + 2 * l_s * s * p[6] + s ** 2 * p[5]
        result[6] = l_s * p[7] + s * p[6]
        result[7] = p[7]

        return result


class Generate_Trajectory(object):
    def __init__(self, waypoints, velocity=1, force_zero_yaw = False):
        wp = waypoints
        yaw = self.get_yaw(wp,force_zero_yaw)
        nodes = self.generate_nodes(wp, yaw)
        segments_time = self.get_segments_time(wp, velocity)
        segments = self.generate_segments(nodes, segments_time)
        self.poly_coef = self.get_polynom_coeff(segments)
 
    def cntl_pnt(self, current, next):
        dic_vec = next - current
        size = np.linalg.norm(dic_vec, ord=2)
        normilized_dir_vec = dic_vec / size
        size_cp = size / 8
        return current + normilized_dir_vec * size_cp #determine the position of the first control point

    def get_segments_time(self, wp, velocity=1):
        segment_time = []
        for i in range(len(wp)-1):
            dist = np.linalg.norm(wp[i+1]-wp[i], ord=2)
            t = dist / velocity
            segment_time.append(t)
        return segment_time

    def get_yaw(self, wp, force_zero_yaw):
        yaws = []
        if force_zero_yaw:
            for i in range(len(wp)-1):
                yaws.append(0)
            return yaws 
        for i in range(len(wp)-1):
            dx = wp[i+1][0] - wp[i][0]
            dy = wp[i+1][1]- wp[i][1]
            if dx < 0:
               yaw = np.arctan2(-dy,-dx) 
            else:
                yaw = np.arctan2(dy,dx)
            yaws.append(yaw)
        return yaws


    def generate_nodes(self, wp, yaw):
        # init
        nodes = []
        wp1 = wp[0]
        node1 = Node((wp1[0], wp1[1], wp1[2], yaw[0]))
        nodes.append(node1)
        # middle
        for i in range(1, len(wp)-1):
            wp1 = wp[i]
            next = wp[i+1]
            cp = self.cntl_pnt(wp1, next)
            node = Node((wp1[0], wp1[1], wp1[2], yaw[i]), q1=(cp[0], cp[1], cp[2], yaw[i]))
            nodes.append(node)
        # end
        wp1 = wp[-1]
        node = Node((wp1[0], wp1[1], wp1[2], yaw[-1]))
        nodes.append(node)
        return nodes

    def generate_segments(self, nodes, segments_time):
        segments = []
        for i in range(len(nodes)-1):
            segments.append(Segment(nodes[i], nodes[i+1], segments_time[i]))
        return segments

    def get_polynom_coeff(self, segments): 
        poly_coef = []
        for s in segments:
            poly_coef.append(s.get_coef())
        return poly_coef


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    upload_result = trajectory_mem.write_data_sync()
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration
