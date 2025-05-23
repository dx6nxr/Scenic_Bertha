"""Parser for OpenDRIVE (.xodr) files."""

import abc
from collections import defaultdict
from distutils.command import clean
import itertools
import math
from turtle import color, left
import warnings
import xml.etree.ElementTree as ET

import matplotlib
import numpy as np
from scipy.integrate import quad, solve_ivp
from scipy.optimize import brentq
from shapely.geometry import GeometryCollection, MultiPoint, MultiPolygon, Point, Polygon
from shapely.ops import snap, unary_union

from scenic.core.geometry import (
    averageVectors,
    cleanChain,
    cleanPolygon,
    plotPolygon,
    polygonUnion,
    removeHoles,
)
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.vectors import Vector
from scenic.domains.driving import roads as roadDomain

from scenic.core.distributions import RejectionException


class OpenDriveWarning(UserWarning):
    pass


def warn(message):
    warnings.warn(message, OpenDriveWarning, stacklevel=2)


def buffer_union(polys, tolerance=0.01):
    return polygonUnion(polys, buf=tolerance, tolerance=tolerance)


class Poly3:
    """Cubic polynomial."""

    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def eval_at(self, x):
        return self.a + self.b * x + self.c * x**2 + self.d * x**3

    def grad_at(self, x):
        return self.b + 2 * self.c * x + 3 * self.d * x**2


class Curve:
    """Geometric elements which compose road reference lines.
    See the OpenDRIVE Format Specification for coordinate system details."""

    def __init__(self, x0, y0, hdg, length):
        self.x0 = x0
        self.y0 = y0
        self.hdg = hdg  # In radians counterclockwise, 0 at positive x-axis.
        self.cos_hdg, self.sin_hdg = math.cos(hdg), math.sin(hdg)
        self.length = length

    def to_points(self, num, extra_points=[]):
        """Sample NUM evenly-spaced points from curve.

        Points are tuples of (x, y, s) with (x, y) absolute coordinates
        and s the arc length along the curve. Additional points at s values in
        extra_points are included if they are contained in the curve (unless
        they are extremely close to one of the equally-spaced points).
        """
        s_vals = []
        extras = itertools.chain(extra_points, itertools.repeat(float("inf")))
        next_extra = next(extras)
        last_s = 0
        for s in np.linspace(0, self.length, num=num):
            while next_extra <= s:
                if last_s + 1e-6 < next_extra < s - 1e-6:
                    s_vals.append(next_extra)
                next_extra = next(extras)
            s_vals.append(s)
            last_s = s
        return [self.point_at(s) for s in s_vals]

    @abc.abstractmethod
    def point_at(self, s):
        """Get an (x, y, s) point along the curve at the given s coordinate."""
        return

    @abc.abstractmethod
    def hdg_at(self, s):
        """Get the heading at the given s coordinate."""
        return

    def rel_to_abs(self, point):
        """Convert from relative coordinates of curve to absolute coordinates.
        I.e. rotate counterclockwise by self.hdg and translate by (x0, x1)."""
        x, y, s = point
        return (
            self.x0 + self.cos_hdg * x - self.sin_hdg * y,
            self.y0 + self.sin_hdg * x + self.cos_hdg * y,
            s,
        )


class Cubic(Curve):
    """A curve defined by the cubic polynomial a + bu + cu^2 + du^3.
    The curve starts at (X0, Y0) in direction HDG, with length LENGTH."""

    def __init__(self, x0, y0, hdg, length, a, b, c, d):
        super().__init__(x0, y0, hdg, length)
        self.poly = Poly3(a, b, c, d)
        # Crude upper bound for u (used to bracket u for a given s in point_at)
        if d != 0:
            self.ubound = max(
                2 * abs(c / d), abs(b / d) ** 0.5, (3 *
                                                    length / abs(d)) ** (1 / 3)
            )
        elif c != 0:
            self.ubound = max(abs(b / c), (2 * length / abs(c)) ** 0.5)
        else:
            self.ubound = length / abs(b)

    def arclength(self, u):
        def d_arc(x): return np.sqrt(1 + self.poly.grad_at(x) ** 2)
        return quad(d_arc, 0, u)[0]

    def point_at(self, s):
        # Use Brent's method to find parameter u corresponding to arclength s;
        # (N.B. Brent's method proved to be faster than Newton's and has no potential
        # convergence issues.)
        def root_func(x): return self.arclength(x) - s
        u = float(brentq(root_func, 0, self.ubound))
        pt = (s, self.poly.eval_at(u), s)
        return self.rel_to_abs(pt)

    def hdg_at(self, s):
        """
        Calculates the heading at a specific point along the cubic curve.

        Args:
            s: The distance along the curve from the starting point.

        Returns:
            The heading at the specified point.
        """

        # Find the corresponding parameter u
        u = self.find_u_for_s(s)
        du_du = self.poly.grad_at(u)
        heading = np.arctan(du_du)
        return heading


class ParamCubic(Curve):
    """A curve defined by the parametric equations
    u = a_u + b_up + c_up^2 + d_up^3,
    v = a_v + b_vp + c_vp^2 + d_up^3,
    with p in [0, p_range].
    The curve starts at (X0, Y0) in direction HDG, with length LENGTH."""

    def __init__(self, x0, y0, hdg, length, au, bu, cu, du, av, bv, cv, dv, p_range=1):
        super().__init__(x0, y0, hdg, length)
        self.u_poly = Poly3(au, bu, cu, du)
        self.v_poly = Poly3(av, bv, cv, dv)
        self.p_range = p_range if p_range else 1

    def arclength(self, p):
        def d_arc(x): return math.hypot(
            self.u_poly.grad_at(x), self.v_poly.grad_at(x))
        return quad(d_arc, 0, p)[0]

    def point_at(self, s):
        def root_func(x): return self.arclength(x) - s
        p = float(brentq(root_func, 0, self.p_range))
        pt = (self.u_poly.eval_at(p), self.v_poly.eval_at(p), s)
        return self.rel_to_abs(pt)

    def hdg_at(self, s):
        """
        Calculates the heading at a specific point along the parametric cubic curve.

        Args:
            s: The distance along the curve from the starting point.

        Returns:
            The heading at the specified point.
        """

        p = self.find_p_for_s(s)  # Find the corresponding parameter p
        du_dp = self.u_poly.grad_at(p)
        dv_dp = self.v_poly.grad_at(p)
        heading = np.arctan2(dv_dp, du_dp)
        return heading


class Clothoid(Curve):
    """An Euler spiral with curvature varying linearly between CURV0 and CURV1.
    The spiral starts at (X0, Y0) in direction HDG, with length LENGTH."""

    def __init__(self, x0, y0, hdg, length, curv0, curv1):
        super().__init__(x0, y0, hdg, length)
        # Initial and final curvature.
        self.curv0 = curv0
        self.curv1 = curv1
        self.curve_rate = (curv1 - curv0) / length
        self.a = abs(curv0)
        self.r = 1 / self.a if curv0 != 0 else 1  # value not used if curv0 == 0
        self.ode_init = np.array([x0, y0, hdg])

    def point_at(self, s):
        # Generate a origin-centered clothoid with zero curvature at origin,
        # then translate/rotate the relevant segment.
        # Arcs are just a degenerate clothoid:
        if self.curv0 == self.curv1:
            if self.curv0 == 0:
                pt = (s, 0, s)
            else:
                r = self.r
                th = s * self.a
                if self.curv0 > 0:
                    pt = (r * math.sin(th), r - r * math.cos(th), s)
                else:
                    pt = (r * math.sin(th), -r + r * math.cos(th), s)
            return self.rel_to_abs(pt)
        else:

            def clothoid_ode(s, state):
                x, y, theta = state
                return np.array(
                    [math.cos(theta), math.sin(theta),
                     self.curv0 + (self.curve_rate * s)]
                )

            sol = solve_ivp(clothoid_ode, (0, s), self.ode_init)
            x, y, hdg = sol.y[:, -1]
            return (x, y, s)

    def hdg_at(self, s):
        """
        Calculates the heading at a specific point along the clothoid curve.

        Args:
            s: The distance along the curve from the starting point.

        Returns:
            The heading at the specified point.
        """

        if self.curv0 == self.curv1:
            if self.curv0 == 0:
                return self.hdg
            else:
                # this case causes shit to happen
                curvature_at_s = self.curv0 + \
                    (self.curv1 - self.curv0) * s / self.length
                radius_at_s = 1 / curvature_at_s
                angle_at_s = s / radius_at_s
                hdg = self.hdg + angle_at_s
                # print ("heading of the curve", self.hdg, f"heading at {s}", hdg)
                return hdg
        # Handle general clothoid case
        sol = solve_ivp(self.clothoid_ode, (0, s), self.ode_init)
        _, _, hdg = sol.y[:, -1]
        return hdg


class Line(Curve):
    """A line segment between (x0, y0) and (x1, y1)."""

    def __init__(self, x0, y0, hdg, length):
        super().__init__(x0, y0, hdg, length)
        # Endpoints of line.
        self.x1 = x0 + length * math.cos(hdg)
        self.y1 = y0 + length * math.sin(hdg)

    def point_at(self, s):
        return self.rel_to_abs((s, 0, s))

    def hdg_at(self, s):
        return self.hdg


class Lane:
    def __init__(self, id_, type_, pred=None, succ=None):
        self.id_ = id_
        self.width = []  # List of tuples (Poly3, int) for width and s-offset.
        self.type_ = type_
        self.pred = pred
        self.succ = succ
        self.left_bounds = []  # to be filled in later
        self.right_bounds = []
        self.centerline = []
        self.rightDrivingEdge = []
        self.leftDrivingEdge = []
        self.parent_lane_poly = None

    def width_at(self, s):
        # S here is relative to start of LaneSection this lane is in.
        ind = 0
        while ind + 1 < len(self.width) and self.width[ind + 1][1] <= s:
            ind += 1
        assert self.width[ind][1] <= s, "No matching width entry found."
        w_poly, s_off = self.width[ind]
        w = w_poly.eval_at(s - s_off)
        if w < -1e-6:  # allow for numerical error
            raise RuntimeError("OpenDRIVE lane has negative width")
        return max(w, 0)


class LaneSection:
    def __init__(self, s0, left_lanes={}, right_lanes={}):
        self.s0 = s0
        self.left_lanes = left_lanes
        self.right_lanes = right_lanes
        self.left_lane_ids = sorted(self.left_lanes.keys())
        self.right_lane_ids = sorted(self.right_lanes.keys(), reverse=True)
        self.lanes = dict(list(left_lanes.items()) + list(right_lanes.items()))

    def get_lane(self, id_):
        if id_ in self.left_lanes:
            return self.left_lanes[id_]
        elif id_ in self.right_lanes:
            return self.right_lanes[id_]
        elif id_ == 0:
            return Lane(0, "none")
        else:
            raise RuntimeError("Lane with id", id_, "not found")

    def get_offsets(self, s):
        """Returns dict of lane id and offset from
        reference line of lane boundary at coordinate S along line.
        By convention, left lanes have positive width offset and right lanes
        have negative."""
        assert s >= self.s0, "Input s is before lane start position."
        offsets = {}
        for lane_id in self.left_lane_ids:
            if lane_id - 1 in self.left_lane_ids:
                offsets[lane_id] = offsets[lane_id - 1] + self.left_lanes[
                    lane_id
                ].width_at(s - self.s0)
            else:
                offsets[lane_id] = self.left_lanes[lane_id].width_at(
                    s - self.s0)
        for lane_id in self.right_lane_ids:
            if lane_id + 1 in self.right_lane_ids:
                offsets[lane_id] = offsets[lane_id + 1] - self.right_lanes[
                    lane_id
                ].width_at(s - self.s0)
            else:
                offsets[lane_id] = - \
                    self.right_lanes[lane_id].width_at(s - self.s0)
        return offsets


class RoadLink:
    """Indicates Roads a and b, with ids id_a and id_b respectively, are connected."""

    def __init__(self, id_a, id_b, contact_a, contact_b):
        self.id_a = id_a
        self.id_b = id_b
        # contact_a and contact_b should be of value "start" or "end"
        # and indicate which end of each road is connected to the other.
        self.contact_a = contact_a
        self.contact_b = contact_b


class Junction:
    class Connection:
        def __init__(self, incoming_id, connecting_id, connecting_contact, lane_links):
            self.incoming_id = incoming_id
            # id of connecting road
            self.connecting_id = connecting_id
            # contact point ('start' or 'end') on connecting road
            self.connecting_contact = connecting_contact
            # dict mapping incoming to connecting lane ids (empty = identity mapping)
            self.lane_links = lane_links

    def __init__(self, id_, name):
        self.id_ = id_
        self.name = name
        self.connections = []
        # Ids of roads that are paths within junction:
        self.paths = []
        self.poly = None

    def add_connection(self, incoming_id, connecting_id, connecting_contact, lane_links):
        conn = Junction.Connection(
            incoming_id, connecting_id, connecting_contact, lane_links
        )
        self.connections.append(conn)


class Road:
    def __init__(self, name, id_, length, junction, drive_on_right=True):
        self.name = name
        self.id_ = id_
        self.length = length
        self.junction = junction if junction != "-1" else None
        self.predecessor = None
        self.successor = None
        self.signals = []  # List of Signal objects.
        self.crossings = []
        self.lane_secs = []  # List of LaneSection objects.
        self.ref_line = []  # List of Curve objects defining reference line.
        # NOTE: sec_points, sec_polys, sec_lane_polys should be ordered according to lane_secs.
        # List of lists of points, one for each LaneSection.
        self.sec_points = []
        self.sec_polys = []  # List of Polygons, one for each LaneSections.
        # List of dict of lane id to Polygon for each LaneSection.
        self.sec_lane_polys = []
        # List of lane polygons. Not a dict b/c lane id is not unique along road.
        self.lane_polys = []
        # Each polygon in lane_polys is the union of connected lane section polygons.
        # lane_polys is currently not used.
        # Reference line offset:
        self.offset = []  # List of tuple (Poly3, s-coordinate).
        self.drive_on_right = drive_on_right
        # Used to fill in gaps between roads:
        self.start_bounds_left = {}
        self.start_bounds_right = {}
        self.end_bounds_left = {}
        self.end_bounds_right = {}

        self.remappedStartLanes = None  # hack for handling spurious initial lane sections

    def get_ref_line_offset(self, s):
        if not self.offset:
            return 0
        ind = 0
        while ind + 1 < len(self.offset) and self.offset[ind + 1][1] <= s:
            ind += 1
        poly, s0 = self.offset[ind]
        assert s >= s0
        return poly.eval_at(s - s0)

    def get_ref_points(self, num):
        """Returns list of list of points for each piece of ref_line.
        List of list structure necessary because each piece needs to be
        constructed into Polygon separately then unioned afterwards to avoid
        self-intersecting lines."""
        ref_points = []
        transition_points = [sec.s0 for sec in self.lane_secs[1:]]
        last_s = 0
        for piece in self.ref_line:
            piece_points = piece.to_points(num, extra_points=transition_points)
            assert piece_points, "Failed to get piece points"
            if ref_points:
                last_s = ref_points[-1][-1][2]
                piece_points = [(p[0], p[1], p[2] + last_s)
                                for p in piece_points]
            ref_points.append(piece_points)
            transition_points = [
                s - last_s for s in transition_points if s > last_s]
        return ref_points

    def heading_at(self, point):
        # Convert point to shapely Point.
        point = Point(point.x, point.y)
        for i in range(len(self.lane_secs)):
            ref_points = self.sec_points[i]
            poly = self.sec_polys[i]
            if point.within(poly.buffer(1)):
                lane_id = None
                for id_ in self.sec_lane_polys[i].keys():
                    if point.within(self.sec_lane_polys[i][id_].buffer(1)):
                        lane_id = id_
                        break
                assert lane_id is not None, "Point not found in sec_lane_polys."
                min_dist = float("inf")
                for i in range(len(ref_points)):
                    cur_point = Point(ref_points[i][0], ref_points[i][1])
                    if point.distance(cur_point) < min_dist:
                        closest_idx = i
                if closest_idx >= len(ref_points) - 1:
                    closest_idx = len(ref_points) - 2
                dy = ref_points[closest_idx + 1][1] - \
                    ref_points[closest_idx][1]
                dx = ref_points[closest_idx + 1][0] - \
                    ref_points[closest_idx][0]
                heading = math.atan2(dy, dx)
                # Right lanes have negative lane_id.
                # Flip heading if drive_on_right XOR right lane.
                if self.drive_on_right != (lane_id < 0):
                    heading += math.pi
                # Heading 0 is defined differently between OpenDrive and Scenic(?)
                heading -= math.pi / 2
                return (heading + math.pi) % (2 * math.pi) - math.pi

        raise RuntimeError("Point not found in piece_polys")

    def calc_geometry_for_type(self, lane_types, num, tolerance, calc_gap=False):
        """Given a list of lane types, returns a tuple of:
        - List of lists of points along the reference line, with same indexing as self.lane_secs
        - List of region polygons, with same indexing as self.lane_secs
        - List of dictionary of lane id to polygon, with same indexing as self.lane_secs
        - List of polygons for each lane (not necessarily by id, but respecting lane successor/predecessor)
        - Polygon for entire region.
        If calc_gap=True, fills in gaps between connected roads. This is fairly expensive.
        """
        road_polygons = []
        ref_points = self.get_ref_points(num)
        self.ref_line_points = list(itertools.chain.from_iterable(ref_points))
        cur_lane_polys = {}
        sec_points = []
        sec_polys = []
        sec_lane_polys = []
        lane_polys = []
        last_lefts = None
        last_rights = None
        cur_p = None

        for i in range(len(self.lane_secs)):
            cur_sec = self.lane_secs[i]
            cur_sec_points = []
            if i < len(self.lane_secs) - 1:
                next_sec = self.lane_secs[i + 1]
                s_stop = next_sec.s0
            else:
                s_stop = float("inf")
            left_bounds = defaultdict(list)
            right_bounds = defaultdict(list)
            cur_sec_lane_polys = defaultdict(list)
            cur_sec_polys = []
            end_of_sec = False

            while ref_points and not end_of_sec:
                if not ref_points[0]:
                    ref_points.pop(0)
                if not ref_points or (cur_p and cur_p[2] >= s_stop):
                    # Case 1: We have processed the entire reference line.
                    # Case 2: The s-coordinate has exceeded s_stop, so we should move
                    # onto the next LaneSection.
                    # Either way, we collect all the bound points so far into polygons.
                    end_of_sec = True
                    cur_last_lefts = {}
                    cur_last_rights = {}
                    for id_ in left_bounds:
                        # Polygon for piece of lane:
                        left = left_bounds[id_]
                        right = right_bounds[id_][::-1]
                        bounds = left + right

                        if len(bounds) < 3:
                            continue
                        poly = cleanPolygon(Polygon(bounds), tolerance)
                        if not poly.is_empty:
                            if poly.geom_type == "MultiPolygon":
                                poly = MultiPolygon(
                                    [
                                        p
                                        for p in poly.geoms
                                        if not p.is_empty and p.exterior
                                    ]
                                )
                                cur_sec_polys.extend(poly.geoms)
                            else:
                                cur_sec_polys.append(poly)
                            cur_sec_lane_polys[id_].append(poly)
                        cur_last_lefts[id_] = left_bounds[id_][-1]
                        cur_last_rights[id_] = right_bounds[id_][-1]
                        if i == 0 or not self.start_bounds_left:
                            self.start_bounds_left[id_] = left_bounds[id_][0]
                            self.start_bounds_right[id_] = right_bounds[id_][0]

                    left_bounds = defaultdict(list)
                    right_bounds = defaultdict(list)
                    if cur_last_lefts and cur_last_rights:
                        last_lefts = cur_last_lefts
                        last_rights = cur_last_rights
                else:
                    cur_p = ref_points[0][0]
                    cur_sec_points.append(cur_p)
                    s = min(max(cur_p[2], cur_sec.s0), s_stop - 1e-6)
                    offsets = cur_sec.get_offsets(s)
                    offsets[0] = 0
                    for id_ in offsets:
                        offsets[id_] += self.get_ref_line_offset(s)
                    if len(ref_points[0]) > 1:
                        next_p = ref_points[0][1]
                        tan_vec = (next_p[0] - cur_p[0], next_p[1] - cur_p[1])
                    else:
                        if len(cur_sec_points) >= 2:
                            prev_p = cur_sec_points[-2]
                        else:
                            assert len(sec_points) > 0
                            if sec_points[-1]:
                                assert sec_points[-1][-1] == cur_p
                                prev_p = sec_points[-1][-2]
                            else:
                                prev_p = sec_points[-2][-2]

                        tan_vec = (cur_p[0] - prev_p[0], cur_p[1] - prev_p[1])
                    tan_norm = math.hypot(tan_vec[0], tan_vec[1])
                    assert tan_norm > 1e-10
                    normal_vec = (-tan_vec[1] / tan_norm,
                                  tan_vec[0] / tan_norm)
                    if cur_p[2] < s_stop:
                        # if at end of section, keep current point to be included in
                        # the next section as well; otherwise remove it
                        ref_points[0].pop(0)
                    elif len(ref_points[0]) == 1 and len(ref_points) > 1:
                        # also get rid of point if this is the last point of the current geometry and
                        # and there is another geometry following
                        ref_points[0].pop(0)
                    for id_ in offsets:
                        lane = cur_sec.get_lane(id_)
                        if lane.type_ in lane_types:
                            if id_ > 0:
                                prev_id = id_ - 1
                            else:
                                prev_id = id_ + 1
                            left_bound = [
                                cur_p[0] + normal_vec[0] * offsets[id_],
                                cur_p[1] + normal_vec[1] * offsets[id_],
                            ]
                            right_bound = [
                                cur_p[0] + normal_vec[0] * offsets[prev_id],
                                cur_p[1] + normal_vec[1] * offsets[prev_id],
                            ]
                            if id_ < 0:
                                left_bound, right_bound = right_bound, left_bound
                            halfway = (offsets[id_] + offsets[prev_id]) / 2
                            centerline = [
                                cur_p[0] + normal_vec[0] * halfway,
                                cur_p[1] + normal_vec[1] * halfway,
                            ]
                            rightDrivingEdge = [
                                centerline[0] - normal_vec[0],
                                centerline[1] - normal_vec[1],
                            ]
                            leftDrivingEdge = [
                                centerline[0] + normal_vec[0],
                                centerline[1] + normal_vec[1],
                            ]
                            if not id_ < 0:
                                leftDrivingEdge, rightDrivingEdge = rightDrivingEdge, leftDrivingEdge
                            left_bounds[id_].append(left_bound)
                            right_bounds[id_].append(right_bound)
                            lane.left_bounds.append(left_bound)
                            lane.right_bounds.append(right_bound)
                            lane.centerline.append(centerline)
                            lane.rightDrivingEdge.append(rightDrivingEdge)
                            lane.leftDrivingEdge.append(leftDrivingEdge)
            assert len(cur_sec_points) >= 2, i
            sec_points.append(cur_sec_points)
            sec_polys.append(buffer_union(cur_sec_polys, tolerance=tolerance))
            for id_ in cur_sec_lane_polys:
                poly = buffer_union(
                    cur_sec_lane_polys[id_], tolerance=tolerance)
                cur_sec_lane_polys[id_] = poly
                cur_sec.get_lane(id_).poly = poly
            sec_lane_polys.append(dict(cur_sec_lane_polys))
            next_lane_polys = {}
            for id_ in cur_sec_lane_polys:
                pred_id = cur_sec.get_lane(id_).pred
                if pred_id and pred_id in cur_lane_polys:
                    next_lane_polys[id_] = cur_lane_polys.pop(pred_id) + [
                        cur_sec_lane_polys[id_]
                    ]
                else:
                    next_lane_polys[id_] = [cur_sec_lane_polys[id_]]
            for id_ in cur_lane_polys:
                poly = buffer_union(cur_lane_polys[id_], tolerance=tolerance)
                self.lane_secs[i -
                               1].get_lane(id_).parent_lane_poly = len(lane_polys)
                lane_polys.append(poly)
            cur_lane_polys = next_lane_polys
        for id_ in cur_lane_polys:
            poly = buffer_union(cur_lane_polys[id_], tolerance=tolerance)
            cur_sec.get_lane(id_).parent_lane_poly = len(lane_polys)
            lane_polys.append(poly)
        union_poly = buffer_union(sec_polys, tolerance=tolerance)
        if last_lefts and last_rights:
            self.end_bounds_left.update(last_lefts)
            self.end_bounds_right.update(last_rights)

        # Difference and slightly erode all overlapping polygons
        for i in range(len(sec_polys) - 1):
            if sec_polys[i].overlaps(sec_polys[i + 1]):
                sec_polys[i] = sec_polys[i].difference(
                    sec_polys[i + 1]).buffer(-1e-6)
                assert not sec_polys[i].overlaps(sec_polys[i + 1])

        for polys in sec_lane_polys:
            ids = sorted(polys)  # order adjacent lanes consecutively
            for i in range(len(ids) - 1):
                polyA, polyB = polys[ids[i]], polys[ids[i + 1]]
                if polyA.overlaps(polyB):
                    polys[ids[i]] = polyA.difference(polyB).buffer(-1e-6)
                    assert not polys[ids[i]].overlaps(polyB)

        for i in range(len(lane_polys) - 1):
            if lane_polys[i].overlaps(lane_polys[i + 1]):
                lane_polys[i] = lane_polys[i].difference(
                    lane_polys[i + 1]).buffer(-1e-6)
                assert not lane_polys[i].overlaps(lane_polys[i + 1])

        # Set parent lane polygon references to corrected polygons
        for sec in self.lane_secs:
            for lane in sec.lanes.values():
                parentIndex = lane.parent_lane_poly
                if isinstance(parentIndex, int):
                    lane.parent_lane_poly = lane_polys[parentIndex]

        return (sec_points, sec_polys, sec_lane_polys, lane_polys, union_poly)

    def calculate_geometry(
        self,
        num,
        tolerance,
        calc_gap,
        drivable_lane_types,
        sidewalk_lane_types,
        shoulder_lane_types,
    ):
        # Note: this also calculates self.start_bounds_left, self.start_bounds_right,
        # self.end_bounds_left, self.end_bounds_right
        (
            self.sec_points,
            self.sec_polys,
            self.sec_lane_polys,
            self.lane_polys,
            self.drivable_region,
        ) = self.calc_geometry_for_type(
            drivable_lane_types, num, tolerance, calc_gap=calc_gap
        )

        for i, sec in enumerate(self.lane_secs):
            sec.drivable_lanes = {}
            sec.sidewalk_lanes = {}
            sec.shoulder_lanes = {}
            for id_, lane in sec.lanes.items():
                ty = lane.type_
                if ty in drivable_lane_types:
                    sec.drivable_lanes[id_] = lane
                elif ty in sidewalk_lane_types:
                    sec.sidewalk_lanes[id_] = lane
                elif ty in shoulder_lane_types:
                    sec.shoulder_lanes[id_] = lane
            if not sec.drivable_lanes:
                continue

            rightmost = None
            for id_ in itertools.chain(reversed(sec.right_lane_ids), sec.left_lane_ids):
                if id_ in sec.drivable_lanes:
                    rightmost = sec.lanes[id_]
                    break
            assert rightmost is not None, i
            leftmost = None
            for id_ in itertools.chain(reversed(sec.left_lane_ids), sec.right_lane_ids):
                if id_ in sec.drivable_lanes:
                    leftmost = sec.lanes[id_]
                    break
            assert leftmost is not None, i
            sec.left_edge = leftmost.left_bounds
            assert len(sec.left_edge) >= 2
            sec.right_edge = rightmost.right_bounds
            assert len(sec.right_edge) >= 2
            sec.right_driving_edge = rightmost.rightDrivingEdge
            assert len(sec.right_driving_edge) >= 2
            sec.left_driving_edge = rightmost.leftDrivingEdge
            assert len(sec.left_driving_edge) >= 2

        _, _, _, _, self.sidewalk_region = self.calc_geometry_for_type(
            sidewalk_lane_types, num, tolerance, calc_gap=calc_gap
        )

        _, _, _, _, self.shoulder_region = self.calc_geometry_for_type(
            shoulder_lane_types, num, tolerance, calc_gap=calc_gap
        )

    def toScenicRoad(self, tolerance):
        assert self.sec_points
        allElements = []
        # Create lane and road sections
        roadSections = []
        last_section = None
        sidewalkSections = defaultdict(list)
        shoulderSections = defaultdict(list)
        for sec, pts, sec_poly, lane_polys in zip(
            self.lane_secs, self.sec_points, self.sec_polys, self.sec_lane_polys
        ):
            pts = [pt[:2] for pt in pts]  # drop s coordinate
            assert sec.drivable_lanes
            laneSections = {}
            for id_, lane in sec.drivable_lanes.items():
                succ = None  # will set this later
                if last_section and lane.pred:
                    if lane.pred in last_section.lanesByOpenDriveID:
                        pred = last_section.lanesByOpenDriveID[lane.pred]
                    else:
                        warn(
                            f"road {self.id_} section {len(roadSections)} "
                            f"lane {id_} has a non-drivable predecessor"
                        )
                        pred = None
                else:
                    pred = lane.pred  # will correct inter-road links later
                left, center, right, rightEdge, leftEdge = lane.left_bounds, lane.centerline, lane.right_bounds, lane.rightDrivingEdge, lane.leftDrivingEdge
                if id_ > 0:  # backward lane
                    left, center, right, rightEdge, leftEdge = right[::-
                                                                     1], center[::-1], left[::-1], rightEdge[::-1], leftEdge[::-1]
                    succ, pred = pred, succ
                section = roadDomain.LaneSection(
                    id=f"road{self.id_}_sec{len(roadSections)}_lane{id_}",
                    polygon=lane_polys[id_],
                    centerline=PolylineRegion(cleanChain(center)),
                    leftEdge=PolylineRegion(cleanChain(left)),
                    rightEdge=PolylineRegion(cleanChain(right)),
                    rightDrivingEdge=PolylineRegion(cleanChain(rightEdge)),
                    leftDrivingEdge=PolylineRegion(cleanChain(leftEdge)),
                    successor=succ,
                    predecessor=pred,
                    lane=None,  # will set these later
                    group=None,
                    road=None,
                    openDriveID=id_,
                    isForward=id_ < 0,
                )
                section._original_lane = lane
                laneSections[id_] = section
                allElements.append(section)
            section = roadDomain.RoadSection(
                id=f"road{self.id_}_sec{len(roadSections)}",
                polygon=sec_poly,
                centerline=PolylineRegion(cleanChain(pts)),
                leftEdge=PolylineRegion(cleanChain(sec.left_edge)),
                rightEdge=PolylineRegion(cleanChain(sec.right_edge)),
                rightDrivingEdge=PolylineRegion(
                    cleanChain(sec.right_driving_edge)),
                leftDrivingEdge=PolylineRegion(
                    cleanChain(sec.left_driving_edge)),
                successor=None,
                predecessor=last_section,
                road=None,  # will set later
                lanesByOpenDriveID=laneSections,
            )
            roadSections.append(section)
            allElements.append(section)
            last_section = section

            for id_, lane in sec.sidewalk_lanes.items():
                sidewalkSections[id_].append(lane)
            for id_, lane in sec.shoulder_lanes.items():
                shoulderSections[id_].append(lane)

        # Build sidewalks and shoulders
        # TODO improve this!
        forwardSidewalks, backwardSidewalks = [], []
        forwardShoulders, backwardShoulders = [], []
        for id_ in sidewalkSections:
            (forwardSidewalks if id_ < 0 else backwardSidewalks).append(id_)
        for id_ in shoulderSections:
            (forwardShoulders if id_ < 0 else backwardShoulders).append(id_)

        def combineSections(laneIDs, sections, name):
            leftmost, rightmost = max(laneIDs), min(laneIDs)
            if len(laneIDs) != leftmost - rightmost + 1:
                warn(f"ignoring {name} in the middle of road {self.id_}")
            leftPoints, rightPoints = [], []
            if leftmost < 0:
                leftmost = rightmost
                while leftmost + 1 in laneIDs:
                    leftmost = leftmost + 1
                leftSecs, rightSecs = sections[leftmost], sections[rightmost]
                for leftSec, rightSec in zip(leftSecs, rightSecs):
                    leftPoints.extend(leftSec.left_bounds)
                    rightPoints.extend(rightSec.right_bounds)
            else:
                rightmost = leftmost
                while rightmost - 1 in laneIDs:
                    rightmost = rightmost - 1
                leftSecs = reversed(sections[leftmost])
                rightSecs = reversed(sections[rightmost])
                for leftSec, rightSec in zip(leftSecs, rightSecs):
                    leftPoints.extend(reversed(rightSec.right_bounds))
                    rightPoints.extend(reversed(leftSec.left_bounds))
            leftEdge = PolylineRegion(cleanChain(leftPoints))
            rightEdge = PolylineRegion(cleanChain(rightPoints))

            # Heuristically create some kind of reasonable centerline
            if len(leftPoints) == len(rightPoints):
                centerPoints = list(
                    averageVectors(l, r) for l, r in zip(leftPoints, rightPoints)
                )
            else:
                num = max(len(leftPoints), len(rightPoints))
                centerPoints = []
                for d in np.linspace(0, 1, num):
                    l = leftEdge.lineString.interpolate(d, normalized=True)
                    r = rightEdge.lineString.interpolate(d, normalized=True)
                    centerPoints.append(
                        averageVectors(l.coords[0], r.coords[0]))
            centerline = PolylineRegion(cleanChain(centerPoints))
            allPolys = (
                sec.poly
                for id_ in range(rightmost, leftmost + 1)
                for sec in sections[id_]
            )
            union = buffer_union(allPolys, tolerance=tolerance)
            id_ = f"road{self.id_}_{name}({leftmost},{rightmost})"
            return id_, union, centerline, leftEdge, rightEdge

        def makeSidewalk(laneIDs):
            if not laneIDs:
                return None
            id_, union, centerline, leftEdge, rightEdge = combineSections(
                laneIDs, sidewalkSections, "sidewalk"
            )
            sidewalk = roadDomain.Sidewalk(
                id=id_,
                polygon=union,
                centerline=centerline,
                leftEdge=leftEdge,
                rightEdge=rightEdge,
                road=None,
                crossings=(),  # TODO add crosswalks
            )
            allElements.append(sidewalk)
            return sidewalk

        forwardSidewalk = makeSidewalk(forwardSidewalks)
        backwardSidewalk = makeSidewalk(backwardSidewalks)

        def makeShoulder(laneIDs):
            if not laneIDs:
                return None
            id_, union, centerline, leftEdge, rightEdge = combineSections(
                laneIDs, shoulderSections, "shoulder"
            )
            shoulder = roadDomain.Shoulder(
                id=id_,
                polygon=union,
                centerline=centerline,
                leftEdge=leftEdge,
                rightEdge=rightEdge,
                road=None,
            )
            allElements.append(shoulder)
            return shoulder

        forwardShoulder = makeShoulder(forwardShoulders)
        backwardShoulder = makeShoulder(backwardShoulders)

        # Connect sections to their successors
        next_section = None
        for sec, section in reversed(list(zip(self.lane_secs, roadSections))):
            if next_section is None:
                next_section = section
                for id_, lane in sec.drivable_lanes.items():
                    newLane = section.lanesByOpenDriveID[id_]
                    if newLane.isForward:
                        # will correct inter-road links later
                        newLane._successor = lane.succ
                    else:
                        newLane._predecessor = lane.succ
                continue
            section._successor = next_section
            for id_, lane in sec.drivable_lanes.items():
                newLane = section.lanesByOpenDriveID[id_]
                if newLane.isForward:
                    newLane._successor = next_section.lanesByOpenDriveID.get(
                        lane.succ)
                else:
                    newLane._predecessor = next_section.lanesByOpenDriveID.get(
                        lane.succ)
            next_section = section

        # Connect lane sections to adjacent lane sections
        for section in roadSections:
            lanes = section.lanesByOpenDriveID
            for id_, lane in lanes.items():
                if id_ < -1:
                    leftID = id_ + 1
                elif id_ == -1:
                    leftID = 1
                elif id_ == 1:
                    leftID = -1
                else:
                    leftID = id_ - 1
                rightID = id_ - 1 if id_ < 0 else id_ + 1
                lane._laneToLeft = lanes.get(leftID)
                lane._laneToRight = lanes.get(rightID)
                if self.drive_on_right:
                    lane._fasterLane = lane._laneToLeft
                    lane._slowerLane = lane._laneToRight
                else:
                    lane._slowerLane = lane._laneToLeft
                    lane._fasterLane = lane._laneToRight
                if lane._fasterLane and lane._fasterLane.isForward != lane.isForward:
                    lane._fasterLane = None
                if lane._slowerLane and lane._slowerLane.isForward != lane.isForward:
                    lane._slowerLane = None
                adj = []
                if lane._laneToLeft:
                    adj.append(lane._laneToLeft)
                if lane._laneToRight:
                    adj.append(lane._laneToRight)
                lane.adjacentLanes = tuple(adj)

        # Gather lane sections into lanes
        nextID = 0
        forwardLanes, backwardLanes = [], []
        for roadSection in roadSections:
            for laneSection in roadSection.lanes:
                laneSection._visited = False
        for roadSection, sec in zip(roadSections, self.lane_secs):
            for laneSection in roadSection.lanes:
                if not laneSection._visited:  # start of new lane
                    forward = laneSection.isForward
                    sections = []
                    successorLane = None  # lane this one will merge into
                    while True:
                        sections.append(laneSection)
                        laneSection._visited = True
                        assert laneSection.isForward == forward
                        if forward:
                            nextSection = laneSection._successor
                        else:
                            nextSection = laneSection._predecessor
                        if not nextSection or not isinstance(
                            nextSection, roadDomain.LaneSection
                        ):
                            break
                        elif nextSection._visited:
                            successorLane = nextSection.lane
                            break
                        laneSection = nextSection
                    ls = laneSection._original_lane
                    assert ls.parent_lane_poly

                    if not forward:
                        sections = tuple(reversed(sections))
                    leftPoints, rightPoints, centerPoints, rightDrivingEdge, leftDrivingEdge = [], [], [], [], []
                    for section in sections:
                        leftPoints.extend(section.leftEdge.points)
                        rightPoints.extend(section.rightEdge.points)
                        centerPoints.extend(section.centerline.points)
                        rightDrivingEdge.extend(
                            section.rightDrivingEdge.points)
                        leftDrivingEdge.extend(section.leftDrivingEdge.points)

                    leftEdge = PolylineRegion(cleanChain(leftPoints))
                    rightEdge = PolylineRegion(cleanChain(rightPoints))
                    centerline = PolylineRegion(cleanChain(centerPoints))
                    rightDrivingEdge = PolylineRegion(
                        cleanChain(rightDrivingEdge))
                    leftDrivingEdge = PolylineRegion(
                        cleanChain(leftDrivingEdge))
                    lane = roadDomain.Lane(
                        id=f"road{self.id_}_lane{nextID}",
                        polygon=ls.parent_lane_poly,
                        centerline=centerline,
                        leftEdge=leftEdge,
                        rightEdge=rightEdge,
                        group=None,
                        road=None,
                        sections=tuple(sections),
                        rightDrivingEdge=rightDrivingEdge,
                        leftDrivingEdge=leftDrivingEdge,
                        successor=successorLane,  # will correct inter-road links later
                    )
                    nextID += 1
                    for section in sections:
                        section.lane = lane
                    (forwardLanes if forward else backwardLanes).append(lane)
                    allElements.append(lane)
        lanes = forwardLanes + backwardLanes
        assert lanes

        # Compute lane adjacencies
        for lane in lanes:
            adj = []
            for section in lane.sections:
                adj.extend(sec.lane for sec in section.adjacentLanes)
            lane.adjacentLanes = tuple(adj)

        # Create lane groups
        def getEdges(forward):
            if forward:
                sec = roadSections[0]
                startLanes = sec.forwardLanes
            else:
                sec = roadSections[-1]
                startLanes = sec.backwardLanes
            leftPoints, leftDrivingEdgePoints = [], []
            current = startLanes[-1]  # get leftmost lane of the first section
            while current and isinstance(current, roadDomain.LaneSection):
                if current._laneToLeft and current._laneToLeft.isForward == forward:
                    current = current._laneToLeft
                leftPoints.extend(current.leftEdge.points)
                leftDrivingEdgePoints.extend(current.leftDrivingEdge.points)
                current = current._successor
            leftEdge = PolylineRegion(cleanChain(leftPoints))
            leftDrivingEdge = PolylineRegion(cleanChain(leftDrivingEdgePoints))
            rightPoints, rightDrivingEdgePoints = [], []
            current = startLanes[0]  # get rightmost lane of the first section
            while current and isinstance(current, roadDomain.LaneSection):
                if current._laneToRight and current._laneToRight.isForward == forward:
                    current = current._laneToRight
                rightPoints.extend(current.rightEdge.points)
                rightDrivingEdgePoints.extend(current.rightDrivingEdge.points)
                current = current._successor
            rightEdge = PolylineRegion(cleanChain(rightPoints))
            rightDrivingEdge = PolylineRegion(
                cleanChain(rightDrivingEdgePoints))
            # rather arbitrary
            middleLane = startLanes[len(startLanes) // 2].lane
            return leftEdge, middleLane.centerline, rightEdge, rightDrivingEdge, leftDrivingEdge

        if forwardLanes:
            leftEdge, centerline, rightEdge, rightDrivingEdge, leftDrivingEdge = getEdges(
                forward=True)
            forwardGroup = roadDomain.LaneGroup(
                id=f"road{self.id_}_forward",
                polygon=buffer_union(
                    (lane.polygon for lane in forwardLanes), tolerance=tolerance
                ),
                centerline=centerline,
                leftEdge=leftEdge,
                rightEdge=rightEdge,
                rightDrivingEdge=leftDrivingEdge,
                leftDrivingEdge=rightDrivingEdge,
                road=None,
                lanes=tuple(forwardLanes),
                curb=(forwardShoulder.rightEdge if forwardShoulder else rightEdge),
                sidewalk=forwardSidewalk,
                bikeLane=None,
                shoulder=forwardShoulder,
                opposite=None,
            )
            allElements.append(forwardGroup)
        else:
            forwardGroup = None
        if backwardLanes:
            leftEdge, centerline, rightEdge, rightDrivingEdge, leftDrivingEdge = getEdges(
                forward=False)
            backwardGroup = roadDomain.LaneGroup(
                id=f"road{self.id_}_backward",
                polygon=buffer_union(
                    (lane.polygon for lane in backwardLanes), tolerance=tolerance
                ),
                centerline=centerline,
                leftEdge=leftEdge,
                rightEdge=rightEdge,
                rightDrivingEdge=rightDrivingEdge,
                leftDrivingEdge=leftDrivingEdge,
                road=None,
                lanes=tuple(backwardLanes),
                curb=(backwardShoulder.rightEdge if backwardShoulder else rightEdge),
                sidewalk=backwardSidewalk,
                bikeLane=None,
                shoulder=backwardShoulder,
                opposite=forwardGroup,
            )
            allElements.append(backwardGroup)
            if forwardGroup:
                forwardGroup._opposite = backwardGroup
        else:
            backwardGroup = None

        # Create signal
        roadSignals = []
        for i, signal_ in enumerate(self.signals):
            signal = roadDomain.Signal(
                uid=f"signal{signal_.id_}_{self.id_}_{i}",
                openDriveID=signal_.id_,
                country=signal_.country,
                type=signal_.type_,
            )
            roadSignals.append(signal)

        # Create road
        assert forwardGroup or backwardGroup
        if forwardGroup:
            rightEdge = forwardGroup.rightEdge
            rightDrivingEdge = forwardGroup.rightDrivingEdge
        else:
            rightEdge = backwardGroup.leftEdge
            rightDrivingEdge = backwardGroup.leftDrivingEdge
        if backwardGroup:
            leftEdge = backwardGroup.rightEdge
            leftDrivingEdge = backwardGroup.rightDrivingEdge
        else:
            leftEdge = forwardGroup.leftEdge
            leftDrivingEdge = forwardGroup.leftDrivingEdge
        centerline = PolylineRegion(
            tuple(pt[:2] for pt in self.ref_line_points))

        road = roadDomain.Road(
            name=self.name,
            # need prefix to prevent collisions with intersections
            uid=f"road{self.id_}",
            id=self.id_,
            polygon=self.drivable_region,
            centerline=centerline,
            leftEdge=leftEdge,
            rightEdge=rightEdge,
            rightDrivingEdge=rightDrivingEdge,
            leftDrivingEdge=leftDrivingEdge,
            lanes=lanes,
            forwardLanes=forwardGroup,
            backwardLanes=backwardGroup,
            sections=roadSections,
            signals=tuple(roadSignals),
            crossings=tuple(self.crossings),
        )
        allElements.append(road)

        # Set up parent references
        if forwardGroup:
            forwardGroup.road = road
            if forwardGroup._sidewalk:
                forwardGroup._sidewalk.road = road
            if forwardGroup._shoulder:
                forwardGroup._shoulder.road = road
                forwardGroup._shoulder.group = forwardGroup
        if backwardGroup:
            backwardGroup.road = road
            if backwardGroup._sidewalk:
                backwardGroup._sidewalk.road = road
            if backwardGroup._shoulder:
                backwardGroup._shoulder.road = road
                backwardGroup._shoulder.group = backwardGroup
        for section in roadSections:
            section.road = road
        for lane in forwardLanes:
            lane.group = forwardGroup
            lane.road = road
            for sec in lane.sections:
                sec.group = forwardGroup
                sec.road = road
                del sec._original_lane
        for lane in backwardLanes:
            lane.group = backwardGroup
            lane.road = road
            for sec in lane.sections:
                sec.group = backwardGroup
                sec.road = road
                del sec._original_lane

        return road, allElements


class Signal:
    """Traffic lights, stop signs, etc."""

    def __init__(self, id_, country, type_, subtype, orientation, validity=None):
        self.id_ = id_
        self.country = country
        self.type_ = type_
        self.subtype = subtype
        self.orientation = orientation
        self.validity = validity

    def is_valid(self):
        return self.validity is None or self.validity != [0, 0]


class SignalReference:
    def __init__(self, id_, orientation, validity=None):
        self.id_ = id_
        self.validity = validity
        self.orientation = orientation

    def is_valid(self):
        return self.validity is None or self.validity != [0, 0]


class RoadMap:
    defaultTolerance = 0.05

    def __init__(
        self,
        tolerance=None,
        fill_intersections=True,
        drivable_lane_types=(
            "driving",
            "entry",
            "exit",
            "offRamp",
            "onRamp",
            "connectingRamp",
        ),
        sidewalk_lane_types=("sidewalk",),
        shoulder_lane_types=("shoulder", "parking", "stop", "border"),
        elide_short_roads=False,
    ):
        self.tolerance = self.defaultTolerance if tolerance is None else tolerance
        self.roads = {}
        self.road_links = []
        self.junctions = {}
        self.sec_lane_polys = []
        self.lane_polys = []
        self.intersection_region = None
        self.fill_intersections = fill_intersections
        self.drivable_lane_types = drivable_lane_types
        self.sidewalk_lane_types = sidewalk_lane_types
        self.shoulder_lane_types = shoulder_lane_types
        self.elide_short_roads = elide_short_roads

    def calculate_geometry(self, num, calc_gap=False, calc_intersect=True):
        # If calc_gap=True, fills in gaps between connected roads.
        # If calc_intersect=True, calculates intersection regions.
        # These are fairly expensive.
        for road in self.roads.values():
            road.calculate_geometry(
                num,
                calc_gap=calc_gap,
                tolerance=self.tolerance,
                drivable_lane_types=self.drivable_lane_types,
                sidewalk_lane_types=self.sidewalk_lane_types,
                shoulder_lane_types=self.shoulder_lane_types,
            )
            self.sec_lane_polys.extend(road.sec_lane_polys)
            self.lane_polys.extend(road.lane_polys)

        if calc_gap:
            drivable_polys = []
            sidewalk_polys = []
            shoulder_polys = []
            for road in self.roads.values():
                drivable_poly = road.drivable_region
                sidewalk_poly = road.sidewalk_region
                shoulder_poly = road.shoulder_region
                if not (drivable_poly is None or drivable_poly.is_empty):
                    drivable_polys.append(drivable_poly)
                if not (sidewalk_poly is None or sidewalk_poly.is_empty):
                    sidewalk_polys.append(sidewalk_poly)
                if not (shoulder_poly is None or shoulder_poly.is_empty):
                    shoulder_polys.append(shoulder_poly)

            for link in self.road_links:
                road_a = self.roads[link.id_a]
                road_b = self.roads[link.id_b]
                assert link.contact_a in [
                    "start", "end"], "Invalid link record."
                assert link.contact_b in [
                    "start", "end"], "Invalid link record."
                if link.contact_a == "start":
                    a_sec = road_a.lane_secs[0]
                    a_bounds_left = road_a.start_bounds_left
                    a_bounds_right = road_a.start_bounds_right
                else:
                    a_sec = road_a.lane_secs[-1]
                    a_bounds_left = road_a.end_bounds_left
                    a_bounds_right = road_a.end_bounds_right
                if link.contact_b == "start":
                    b_bounds_left = road_b.start_bounds_left
                    b_bounds_right = road_b.start_bounds_right
                else:
                    b_bounds_left = road_b.end_bounds_left
                    b_bounds_right = road_b.end_bounds_right

                for id_, lane in a_sec.lanes.items():
                    if link.contact_a == "start":
                        other_id = lane.pred
                    else:
                        other_id = lane.succ
                    if other_id not in b_bounds_left or other_id not in b_bounds_right:
                        continue
                    if id_ not in a_bounds_left or id_ not in a_bounds_right:
                        continue

                    gap_poly = MultiPoint(
                        [
                            a_bounds_left[id_],
                            a_bounds_right[id_],
                            b_bounds_left[other_id],
                            b_bounds_right[other_id],
                        ]
                    ).convex_hull
                    if not gap_poly.is_valid:
                        continue
                    if gap_poly.geom_type == "Polygon" and not gap_poly.is_empty:
                        if lane.type_ in self.drivable_lane_types:
                            drivable_polys.append(gap_poly)
                        elif lane.type_ in self.sidewalk_lane_types:
                            sidewalk_polys.append(gap_poly)
                        elif lane.type_ in self.shoulder_lane_types:
                            shoulder_polys.append(gap_poly)
        else:
            drivable_polys = [
                road.drivable_region for road in self.roads.values()]
            sidewalk_polys = [
                road.sidewalk_region for road in self.roads.values()]
            shoulder_polys = [
                road.shoulder_region for road in self.roads.values()]

        self.drivable_region = buffer_union(
            drivable_polys, tolerance=self.tolerance)
        self.sidewalk_region = buffer_union(
            sidewalk_polys, tolerance=self.tolerance)
        self.shoulder_region = buffer_union(
            shoulder_polys, tolerance=self.tolerance)

        if calc_intersect:
            self.calculate_intersections()

    def calculate_intersections(self):
        intersect_polys = []
        for junc in self.junctions.values():
            junc_polys = [self.roads[i].drivable_region for i in junc.paths]
            assert junc_polys, junc
            union = buffer_union(junc_polys, tolerance=self.tolerance)
            if self.fill_intersections:
                union = removeHoles(union)
            assert union.is_valid
            junc.poly = union
            intersect_polys.append(union)
        self.intersection_region = buffer_union(
            intersect_polys, tolerance=self.tolerance)

    def heading_at(self, point):
        """Return the road heading at point."""
        # Convert point to shapely Point.
        point = Point(point.x, point.y)
        for road in self.roads.values():
            if point.within(road.drivable_region.buffer(1)):
                return road.heading_at(point)
        # raise RuntimeError('Point not in RoadMap: ', point)
        return 0

    def __parse_lanes(self, lanes_elem):
        """Lanes_elem should be <left> or <right> element.
        Returns dict of lane ids and Lane objects."""
        lanes = {}
        for l in lanes_elem.iter("lane"):
            id_ = int(l.get("id"))
            type_ = l.get("type")
            link = l.find("link")
            pred = None
            succ = None
            if link is not None:
                pred_elem = link.find("predecessor")
                succ_elem = link.find("successor")
                if pred_elem is not None:
                    pred = int(pred_elem.get("id"))
                if succ_elem is not None:
                    succ = int(succ_elem.get("id"))
            lane = Lane(id_, type_, pred, succ)
            for w in l.iter("width"):
                w_poly = Poly3(
                    float(w.get("a")),
                    float(w.get("b")),
                    float(w.get("c")),
                    float(w.get("d")),
                )
                lane.width.append((w_poly, float(w.get("sOffset"))))
            lanes[id_] = lane
        return lanes

    def __parse_link(self, link_elem, road, contact):
        if link_elem is None:
            return
        road_id = road.id_
        if link_elem.get("elementType") == "road":
            id_b = int(link_elem.get("elementId"))
            contact_b = link_elem.get("contactPoint")
            link = RoadLink(road_id, id_b, contact, contact_b)
            self.road_links.append(link)
            return link
        else:
            assert link_elem.get(
                "elementType") == "junction", "Unknown link type"
            junction = int(link_elem.get("elementId"))
            if junction not in self.junctions:
                return  # junction had no connecting roads, so we skipped it
            if contact == "start":
                road.predecessor = junction
            else:
                road.successor = junction
            connections = self.junctions[junction].connections
            for c in connections:
                if c.incoming_id == road_id:
                    self.road_links.append(
                        RoadLink(road_id, c.connecting_id,
                                 contact, c.connecting_contact)
                    )

    def __parse_signal_validity(self, validity_elem):
        if validity_elem is None:
            return None
        return [int(validity_elem.get("fromLane")), int(validity_elem.get("toLane"))]

    def __parse_signal(self, signal_elem):
        return Signal(
            signal_elem.get("id"),
            signal_elem.get("country"),
            signal_elem.get("type"),
            signal_elem.get("subtype"),
            signal_elem.get("orientation"),
            self.__parse_signal_validity(signal_elem.find("validity")),
        )

    def __parse_crosswalk(self, crosswalk_elem, reference_points):
        """
        Parses a crosswalk element and returns its geometry in global coordinates.

        Args:
            crosswalk_elem: The XML element representing the crosswalk.
            reference_points: A list of reference points (curves) along the road.

        Returns:
            A crosswalk object.
        """
        from math import radians, sin, cos
        # print("Parsing crosswalk", crosswalk_elem.get("id"))
        s0 = float(crosswalk_elem.get("s"))
        t0 = float(crosswalk_elem.get("t"))
        orient = crosswalk_elem.get("orientation")
        width = float(crosswalk_elem.get("width"))

        # Find the appropriate reference point based on the crosswalk's "s" coordinate
        for curve in reference_points:
            if s0 < curve.length:
                _curve = curve
                break
            else:
                s0 -= curve.length

        # Calculate the global coordinates of the crosswalk points
        global_coords = []

        if orient == "+":
            hdg = _curve.hdg_at(s0 + width / 2)
        else:
            hdg = _curve.hdg_at(s0 - width / 2)

        # print (f"Crosswalk {crosswalk_elem.get('id')} hdg", hdg, "elem heading", elemhdg, "hdg + elemhdg", elemhdg - hdg)

        for point in crosswalk_elem.find("outline").findall("cornerLocal"):
            u, v, _ = float(point.get("u")), float(point.get("v")), 0

            # v and u are swapped because crossings are rotated 90 degrees in relation to the road
            if orient == "+":  # add the element in positive s direction
                s = s0 + v
                t = t0 + u
            elif orient == "-":  # add the element in negative s direction
                s = s0 - v
                t = t0 - u

            # Adjust the coordinates based on the element heading
            x, y, _ = _curve.point_at(s)
            x = x + t * sin(hdg)
            y = y + t * cos(hdg)
            global_coords.append((x, y))

        def parse_crosswalk_geometry(global_coords):
            """
            Parses the geometry of a crosswalk, finding the centerline, leftEdge, and rightEdge.

            Args:
                global_coords: A list of global coordinates (x, y) for the crosswalk points.

            Returns:
                A tuple containing the centerline, leftEdge, and rightEdge as LineString objects.
            """

            # Create a Polygon object from the global coordinates
            polygon = Polygon(global_coords)

            # Find the two longest edges (left and right edges)
            left_edge, right_edge = find_longer_edges(polygon)

            # Find the centerline (connection between the centers of the shorter sides)
            centerline = find_centerline(polygon)

            return polygon, centerline, left_edge, right_edge

        def find_longer_edges(polygon):
            """
            Finds the two longest edges of a polygon, handling degenerate cases.

            Args:
                polygon: A Shapely Polygon object.

            Returns:
                A tuple containing the two longest edges as LineString objects, or
                empty LineString objects if no valid edges are found.
            """

            # if polygon.is_empty or polygon.area == 0:
            #    # Handle degenerate polygon
            #    return cleanChain([]), cleanChain([])

            edges = []
            coords = polygon.exterior.coords
            # plot the crosswalk
            # import matplotlib.pyplot as plt
            # plt.plot(*polygon.exterior.xy)
            # plt.show()

            if len(coords) > 5:
                for i in range(len(coords) - 2):
                    A, B, C = coords[i], coords[i + 1], coords[i + 2]
                    angle = calculate_angle(A, B, C)
                    if angle < 15:  # Adjust angle threshold as needed
                        edges.append((A, B, C))
            else:
                return cleanChain([coords[0], coords[1]]), cleanChain([coords[2], coords[3]])

            # Find the two longest edges
            if len(edges) >= 2:
                edges.sort(key=lambda x: calculate_edge_length(x[0], x[1]))
                longest_edges = edges[-2:]
                return cleanChain(longest_edges[0][0:2]), cleanChain(longest_edges[1][0:2])

            return cleanChain([coords[0], coords[1]]), cleanChain([coords[2], coords[3]])

        def find_centerline(polygon):
            """
            Finds the centerline of a crosswalk by connecting the centers of the shorter sides.

            Args:
                polygon: A Shapely Polygon object.

            Returns:
                A LineString object representing the centerline.
            """

            edges = []
            coords = polygon.exterior.coords
            for i in range(len(coords) - 1):
                A, B = coords[i], coords[i + 1]
                distance = calculate_edge_length(A, B)
                edges.append((A, B, distance))

            # Find the two shortest edges
            edges.sort(key=lambda x: x[2])
            shorter_edges = edges[:2]

            # Find the centers of the shorter edges
            centers = []
            for edge in shorter_edges:
                center = ((edge[0][0] + edge[1][0]) / 2,
                          (edge[0][1] + edge[1][1]) / 2)
                centers.append(center)

            return cleanChain(centers)

        def calculate_angle(A, B, C):
            """
            Calculates the angle ABC (in degrees) given three points A, B, and C.

            Args:
                A, B, C: Three points as tuples.

            Returns:
                The angle ABC in degrees.
            """

            AB = np.array(B) - np.array(A)
            BC = np.array(C) - np.array(B)
            cosine_angle = np.dot(AB, BC) / \
                (np.linalg.norm(AB) * np.linalg.norm(BC))
            angle = np.arccos(np.clip(cosine_angle, -1, 1))
            return np.degrees(angle)

        def calculate_edge_length(A, B):
            """
            Calculates the length of an edge between two points.

            Args:
                A, B: Two points as tuples.

            Returns:
                The length of the edge.
            """

            return np.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

        crosswalk_polygon, centerline, left_edge, right_edge = parse_crosswalk_geometry(
            global_coords)
        # print (f"Crosswalk {crosswalk_elem.get('id')} parent {parent.id_}")

        return roadDomain.PedestrianCrossing(
            id=crosswalk_elem.get("id"),
            uid=("Crosswalk" + crosswalk_elem.get("id")),
            name=crosswalk_elem.get("name"),
            polygon=crosswalk_polygon,
            centerline=PolylineRegion(centerline),
            leftEdge=PolylineRegion(left_edge),
            rightEdge=PolylineRegion(right_edge),
            road=None,
            startSidewalk=None,
            endSidewalk=None,
        )

    def __parse_signal_reference(self, signal_reference_elem):
        return SignalReference(
            signal_reference_elem.get("id"),
            signal_reference_elem.get("orientation"),
            self.__parse_signal_validity(
                signal_reference_elem.find("validity")),
        )

    def parse(self, path):
        tree = ET.parse(path)
        root = tree.getroot()
        if root.tag != "OpenDRIVE":
            raise ValueError(f"{path} does not appear to be an OpenDRIVE file")

        # parse junctions
        for j in root.iter("junction"):
            junction = Junction(int(j.get("id")), j.get("name"))
            for c in j.iter("connection"):
                ty = c.get("type", "default")
                if ty != "default":
                    raise NotImplementedError(
                        f'unhandled "{ty}" type of junction connection'
                    )
                lane_links = {}
                for l in c.iter("laneLink"):
                    lane_links[int(l.get("from"))] = int(l.get("to"))
                junction.add_connection(
                    int(c.get("incomingRoad")),
                    int(c.get("connectingRoad")),
                    c.get("contactPoint"),
                    lane_links,
                )
                junction.paths.append(int(c.get("connectingRoad")))
            if not junction.paths:
                warn(
                    f"junction {junction.id_} has no connecting roads; skipping it")
                continue
            self.junctions[junction.id_] = junction

        # Creating temporal signals container to resolve referenced signals.
        _temp_signals = {}
        for r in root.iter("road"):
            signals = r.find("signals")
            if signals is not None:
                for s in signals.iter("signal"):
                    signal = self.__parse_signal(s)
                    _temp_signals[signal.id_] = signal

        # parse roads
        self.elidedRoads = {}
        for r in root.iter("road"):
            road = Road(
                r.get("name"), int(r.get("id")), float(
                    r.get("length")), r.get("junction")
            )
            link = r.find("link")
            if link is not None:
                pred_elem = link.find("predecessor")
                succ_elem = link.find("successor")
                pred_link = self.__parse_link(pred_elem, road, "start")
                succ_link = self.__parse_link(succ_elem, road, "end")
            else:
                pred_link = succ_link = None

            if road.length < self.tolerance:
                warn(
                    f"road {road.id_} has length shorter than tolerance;"
                    " geometry may contain artifacts"
                )
                if self.elide_short_roads:
                    warn(
                        f"attempting to elide road {road.id_} of length {road.length}")
                    assert road.junction is None
                    self.elidedRoads[road.id_] = road
                    if pred_link:
                        road.predecessor = pred_link.id_b
                        road.predecessorContact = pred_link.contact_b
                    else:
                        road.predecessorContact = None
                    if succ_link:
                        road.successor = succ_link.id_b
                        road.successorContact = succ_link.contact_b
                    else:
                        road.successorContact = None
                    continue

            # Parse planView:
            plan_view = r.find("planView")
            curves = []
            for geom in plan_view.iter("geometry"):
                x0 = float(geom.get("x"))
                y0 = float(geom.get("y"))
                s0 = float(geom.get("s"))
                hdg = float(geom.get("hdg"))
                length = float(geom.get("length"))
                curve_elem = geom[0]
                curve = None
                if curve_elem.tag == "line":
                    curve = Line(x0, y0, hdg, length)
                elif curve_elem.tag == "arc":
                    # Arc is clothoid of constant curvature.
                    curv = float(curve_elem.get("curvature"))
                    curve = Clothoid(x0, y0, hdg, length, curv, curv)
                elif curve_elem.tag == "spiral":
                    curv0 = float(curve_elem.get("curvStart"))
                    curv1 = float(curve_elem.get("curvEnd"))
                    curve = Clothoid(x0, y0, hdg, length, curv0, curv1)
                elif curve_elem.tag == "poly3":
                    a, b, c, d = (
                        cubic_elem.get("a"),
                        float(curve_elem.get("b")),
                        float(curve_elem.get("c")),
                        float(curve_elem.get("d")),
                    )
                    curve = Cubic(x0, y0, hdg, length, a, b, c, d)
                elif curve_elem.tag == "paramPoly3":
                    au, bu, cu, du, av, bv, cv, dv = (
                        float(curve_elem.get("aU")),
                        float(curve_elem.get("bU")),
                        float(curve_elem.get("cU")),
                        float(curve_elem.get("dU")),
                        float(curve_elem.get("aV")),
                        float(curve_elem.get("bV")),
                        float(curve_elem.get("cV")),
                        float(curve_elem.get("dV")),
                    )
                    p_range = curve_elem.get("pRange")
                    if p_range and p_range != "normalized":
                        # TODO support arcLength
                        raise NotImplementedError(
                            "unsupported pRange for paramPoly3")
                    else:
                        p_range = 1
                    curve = ParamCubic(
                        x0, y0, hdg, length, au, bu, cu, du, av, bv, cv, dv, p_range
                    )
                curves.append((s0, curve))
            if not curves:
                raise ValueError(f"road {road.id_} has an empty planView")
            if not curves[0][0] == 0:
                raise ValueError(
                    f"reference line of road {road.id_} does not start at s=0"
                )
            lastS = 0
            lastCurve = curves[0][1]
            refLine = []
            for s0, curve in curves[1:]:
                l = s0 - lastS
                if abs(lastCurve.length - l) > 1e-4:
                    raise ValueError(
                        f"planView of road {road.id_} has inconsistent length"
                    )
                if l < 0:
                    raise ValueError(
                        f"planView of road {road.id_} is not in order")
                elif l < 1e-6:
                    warn(
                        f"road {road.id_} reference line has a geometry of "
                        f"length {l}; skipping it"
                    )
                else:
                    refLine.append(lastCurve)
                lastS = s0
                lastCurve = curve
            if refLine and lastCurve.length < 1e-6:
                warn(
                    f"road {road.id_} reference line has a geometry of "
                    f"length {lastCurve.length}; skipping it"
                )
            else:
                # even if the last curve is shorter than the threshold, we'll keep it if
                # it is the only curve; getting rid of the road entirely is handled by
                # road elision above
                refLine.append(lastCurve)
            assert refLine
            road.ref_line = refLine

            # Parse lanes:
            lanes = r.find("lanes")
            for offset in lanes.iter("laneOffset"):
                road.offset.append(
                    (
                        Poly3(
                            float(offset.get("a")),
                            float(offset.get("b")),
                            float(offset.get("c")),
                            float(offset.get("d")),
                        ),
                        float(offset.get("s")),
                    )
                )

            def popLastSectionIfShort(l):
                if l < 1e-6:
                    warn(
                        f"road {road.id_} has a lane section of length {l}; skipping it")

                    # delete the length-0 section and re-link lanes appropriately
                    badSec = road.lane_secs.pop()
                    if road.lane_secs:
                        prev = road.lane_secs[-1]
                        for id_, lane in prev.lanes.items():
                            if lane.succ is not None:
                                lane.succ = badSec.lanes[lane.succ].succ
                    else:
                        if road.remappedStartLanes is None:
                            road.remappedStartLanes = {
                                l: l for l in badSec.lanes}
                        for start, current in road.remappedStartLanes.items():
                            road.remappedStartLanes[start] = badSec.lanes[current].succ
                    return badSec
                else:
                    return None

            last_s = float("-inf")
            for ls_elem in lanes.iter("laneSection"):
                s = float(ls_elem.get("s"))
                l = s - last_s
                assert l >= 0
                badSec = popLastSectionIfShort(l)

                last_s = s
                left = ls_elem.find("left")
                right = ls_elem.find("right")
                left_lanes = {}
                right_lanes = {}

                if left is not None:
                    left_lanes = self.__parse_lanes(left)

                if right is not None:
                    right_lanes = self.__parse_lanes(right)

                lane_sec = LaneSection(s, left_lanes, right_lanes)

                if badSec is not None:  # finish re-linking lanes across deleted section
                    for id_, lane in lane_sec.lanes.items():
                        if lane.pred is not None:
                            lane.pred = badSec.lanes[lane.pred].pred

                road.lane_secs.append(lane_sec)

            # parse signals
            signals = r.find("signals")
            if signals is not None:
                for signal_elem in signals.iter("signal"):
                    signal = self.__parse_signal(signal_elem)
                    if signal.is_valid():
                        road.signals.append(signal)

                for signal_ref_elem in signals.iter("signalReference"):
                    signalReference = self.__parse_signal_reference(
                        signal_ref_elem)
                    if signalReference.is_valid():
                        referencedSignal = _temp_signals[signalReference.id_]
                        signal = Signal(
                            referencedSignal.id_,
                            referencedSignal.country,
                            referencedSignal.type_,
                            referencedSignal.subtype,
                            signalReference.orientation,
                            signalReference.validity,
                        )
                        road.signals.append(signal)

            # added code for parsing the crosswalks
            objs = r.find("objects")
            if objs is not None:
                for crosswalk_elem in objs.iter("object"):
                    if crosswalk_elem.get("type") == "crosswalk":
                        crosswalk = self.__parse_crosswalk(
                            crosswalk_elem, road.ref_line)
                        road.crossings.append(crosswalk)

            if len(road.lane_secs) > 1:
                popLastSectionIfShort(road.length - s)
            assert road.lane_secs
            self.roads[road.id_] = road

        # Handle links to/from elided roads
        new_links = []
        for link in self.road_links:
            if link.id_a in self.elidedRoads:
                continue
            if link.id_b in self.elidedRoads:
                elided = self.elidedRoads[link.id_b]
                if link.contact_b == "start":
                    link.id_b = elided.successor
                    link.contact_b = elided.successorContact
                else:
                    link.id_b = elided.predecessor
                    link.contact_b = elided.predecessorContact
                if link.contact_b is None:
                    continue  # link to intersection
            new_links.append(link)
        self.road_links = new_links

    def toScenicNetwork(self):
        assert self.intersection_region is not None

        # Prepare registry of network elements
        allElements = {}

        def register(element):
            assert element.uid is not None
            assert element.uid not in allElements, element.uid
            allElements[element.uid] = element

        def registerAll(elements):
            for elt in elements:
                register(elt)

        # Convert roads
        mainRoads, connectingRoads, roads = {}, {}, {}
        for id_, road in self.roads.items():
            if road.drivable_region.is_empty:
                continue  # not actually a road you can drive on
            newRoad, elts = road.toScenicRoad(tolerance=self.tolerance)
            registerAll(elts)
            (connectingRoads if road.junction else mainRoads)[id_] = newRoad
            roads[id_] = newRoad

        # Hook up inter-road links
        for link in self.road_links:
            if link.id_b in connectingRoads:
                continue  # actually a road-to-junction link; handled later
            if link.id_a not in roads or link.id_b not in roads:
                continue  # may link non-drivable roads we haven't parsed; ignore it

            # Work out connectivity of roads and adjacent sections
            roadA, roadB = roads[link.id_a], roads[link.id_b]
            if link.contact_a == "start":
                secA = roadA.sections[0]
                roadA._predecessor = roadB
                forwardA = True
            else:
                secA = roadA.sections[-1]
                roadA._successor = roadB
                forwardA = False
            if link.contact_b == "start":
                secB = roadB.sections[0]
            else:
                secB = roadB.sections[-1]

            # Connect corresponding lanes
            lanesB = secB.lanesByOpenDriveID
            for laneA in secA.lanes:
                if laneA.isForward == forwardA:
                    pred = laneA._predecessor
                    if pred is None:
                        continue
                    assert pred in lanesB
                    laneB = lanesB[pred]
                    laneA._predecessor = laneB
                    laneA.lane._predecessor = laneB.lane
                    laneA.lane.group._predecessor = laneB.lane.group
                else:
                    succ = laneA._successor
                    if succ is None:
                        continue
                    assert succ in lanesB
                    laneB = lanesB[succ]
                    laneA._successor = laneB
                    laneA.lane._successor = laneB.lane
                    laneA.lane.group._successor = laneB.lane.group

        # Hook up connecting road links and create intersections
        intersections = {}
        for jid, junction in self.junctions.items():
            if not junction.connections:
                continue
            assert junction.poly is not None
            if junction.poly.is_empty:
                warn(f"skipping empty junction {jid}")
                continue

            # Gather all lanes involved in the junction's connections
            allIncomingLanes, allOutgoingLanes = [], []
            allRoads, seenRoads = [], set()
            allCrossings, seenCrossings = [], set()
            allSignals, seenSignals = [], set()
            maneuversForLane = defaultdict(list)
            for connection in junction.connections:
                incomingID = connection.incoming_id
                incomingRoad = mainRoads.get(incomingID)
                if not incomingRoad:
                    continue  # incoming road has no drivable lanes; skip it

                connectingID = connection.connecting_id
                connectingRoad = connectingRoads.get(connectingID)
                if not connectingRoad:
                    continue  # connecting road has no drivable lanes; skip it

                for signal in connectingRoad.signals:
                    if signal.openDriveID not in seenSignals:
                        allSignals.append(signal)
                        seenSignals.add(signal.openDriveID)
                for crossing in connectingRoad.crossings:
                    if crossing.id not in seenCrossings:
                        allCrossings.append(crossing)
                        seenCrossings.add(crossing.id)

                # Find possible incoming lanes for this connection
                if incomingID not in seenRoads:
                    allRoads.append(incomingRoad)
                    seenRoads.add(incomingID)
                oldRoad = self.roads[incomingID]
                incomingSection = None
                if oldRoad.predecessor == jid:
                    incomingSection = incomingRoad.sections[0]
                    # could be None
                    remapping = self.roads[incomingID].remappedStartLanes
                if oldRoad.successor == jid:
                    assert incomingSection is None
                    incomingSection = incomingRoad.sections[-1]
                    remapping = None
                assert incomingSection is not None
                if remapping is None:
                    incomingLaneIDs = incomingSection.lanesByOpenDriveID
                else:
                    incomingLaneIDs = {}
                    newIDs = incomingSection.lanesByOpenDriveID
                    for start, remapped in remapping.items():
                        if remapped in newIDs:
                            incomingLaneIDs[start] = newIDs[remapped]
                    assert len(incomingLaneIDs) == len(newIDs)

                for road in allRoads:
                    for crossing in road.crossings:
                        if crossing.id not in seenCrossings:
                            allCrossings.append(crossing)
                            seenCrossings.add(crossing.id)

                # Connect incoming lanes to connecting road
                if connection.connecting_contact == "start":
                    connectingSection = connectingRoad.sections[0]
                    remapping = self.roads[
                        connectingID
                    ].remappedStartLanes  # could be None
                else:
                    connectingSection = connectingRoad.sections[-1]
                    remapping = None
                if remapping is None:
                    connectingLaneIDs = connectingSection.lanesByOpenDriveID
                else:
                    connectingLaneIDs = {}
                    newIDs = connectingSection.lanesByOpenDriveID
                    for start, remapped in remapping.items():
                        if remapped in newIDs:
                            connectingLaneIDs[start] = newIDs[remapped]
                    assert len(connectingLaneIDs) == len(newIDs)
                lane_links = connection.lane_links
                if not lane_links:  # all lanes connect to that with the same id
                    lane_links = {l: l for l in incomingLaneIDs}
                for fromID, fromLane in incomingLaneIDs.items():
                    # Link incoming lane to connecting road
                    # (we only handle lanes in incomingLaneIDs, thus skipping non-drivable lanes)
                    if fromID not in lane_links:
                        continue  # lane not linked by this connection
                    toID = lane_links[fromID]
                    toLane = connectingLaneIDs[toID]
                    if fromLane.lane not in allIncomingLanes:
                        allIncomingLanes.append(fromLane.lane)
                    fromLane._successor = toLane
                    fromLane.lane._successor = toLane.lane
                    toLane._predecessor = fromLane
                    toLane.lane._predecessor = fromLane.lane

                    # Collect outgoing lane and road
                    # TODO why is it allowed for this not to exist?
                    outgoingLane = toLane.lane._successor
                    if outgoingLane is None:
                        warn(
                            f"connecting road {connectingID} lane {toID} has no successor lane"
                        )
                    else:
                        if outgoingLane not in allOutgoingLanes:
                            allOutgoingLanes.append(outgoingLane)
                        outgoingRoad = outgoingLane.road
                        if outgoingRoad.id not in seenRoads:
                            allRoads.append(outgoingRoad)
                            seenRoads.add(outgoingRoad.id)

                        # TODO future OpenDRIVE extension annotating left/right turns?
                        maneuver = roadDomain.Maneuver(
                            startLane=fromLane.lane,
                            connectingLane=toLane.lane,
                            endLane=outgoingLane,
                            intersection=None,  # will be patched once the Intersection is created
                        )

                        if maneuver.connectingLane.road.crossings:
                            # check the distance between the crossing an the start/end lane
                            # assign the crossing to the nearest lane road
                            connectingRoad = maneuver.connectingLane.road
                            for crossing in connectingRoad.crossings:
                                startDistance = crossing.polygon.distance(
                                    maneuver.startLane.polygon)
                                endDistance = crossing.polygon.distance(
                                    maneuver.endLane.polygon)
                                if startDistance < endDistance:
                                    # append the crossing to the start lane
                                    crossings = list(
                                        maneuver.startLane.road.crossings)
                                    crossings.append(crossing)
                                    maneuver.startLane.road.crossings = tuple(
                                        crossings)
                                    # remove the crossing from the connecting road
                                    crossings = list(connectingRoad.crossings)
                                    crossings.remove(crossing)
                                    object.__setattr__(
                                        connectingRoad, "crossings", tuple(crossings))
                                else:
                                    # append the crossing to the end lane
                                    crossings = list(
                                        maneuver.endLane.road.crossings)
                                    crossings.append(crossing)
                                    maneuver.endLane.road.crossings = tuple(
                                        crossings)
                                    # remove the crossing from the connecting road
                                    crossings = list(connectingRoad.crossings)
                                    crossings.remove(crossing)
                                    object.__setattr__(
                                        connectingRoad, "crossings", tuple(crossings))

                        maneuversForLane[fromLane.lane].append(maneuver)

            # Gather maneuvers
            allManeuvers = []
            for lane, maneuvers in maneuversForLane.items():
                assert lane.maneuvers == ()
                lane.maneuvers = tuple(maneuvers)
                allManeuvers.extend(maneuvers)

            # Order connected roads and lanes by adjacency
            def cyclicOrder(elements, contactStart=None):
                points = []
                for element in elements:
                    if contactStart is None:
                        old = self.roads[element.id]
                        assert old.predecessor == jid or old.successor == jid
                        contactStart = old.predecessor == jid
                    point = element.centerline[0 if contactStart else -1]
                    points.append(point)
                centroid = sum(points, Vector(0, 0)) / len(points)
                pairs = sorted(
                    zip(elements, points), key=lambda pair: centroid.angleTo(pair[1])
                )
                return tuple(elem for elem, pt in pairs)

            # Create intersection
            intersection = roadDomain.Intersection(
                polygon=junction.poly,
                name=junction.name,
                # need prefix to prevent collisions with roads
                uid=f"intersection{jid}",
                id=jid,
                roads=cyclicOrder(allRoads),
                incomingLanes=cyclicOrder(
                    allIncomingLanes, contactStart=False),
                outgoingLanes=cyclicOrder(allOutgoingLanes, contactStart=True),
                maneuvers=tuple(allManeuvers),
                signals=tuple(allSignals),
                crossings=tuple(allCrossings),  # TODO add these
            )
            register(intersection)
            intersections[jid] = intersection
            for maneuver in allManeuvers:
                object.__setattr__(maneuver, "intersection", intersection)

        # Hook up road-intersection links
        for rid, oldRoad in self.roads.items():
            if rid not in roads:
                continue  # road does not have any drivable lanes, so we skipped it
            newRoad = roads[rid]
            if oldRoad.predecessor:
                intersection = intersections[oldRoad.predecessor]
                newRoad._predecessor = intersection
                newRoad.sections[0]._predecessor = intersection
                if newRoad.backwardLanes:
                    newRoad.backwardLanes._successor = intersection
            if oldRoad.successor:
                intersection = intersections[oldRoad.successor]
                newRoad._successor = intersection
                newRoad.sections[-1]._successor = intersection
                if newRoad.forwardLanes:
                    newRoad.forwardLanes._successor = intersection

        # Gather all network elements
        roads = tuple(mainRoads.values())
        for road in roads:
            for crossing in road.crossings:
                crossing.road = road
                if road.forwardLanes:
                    if road.forwardLanes._sidewalk:
                        object.__setattr__(
                            crossing, "startSidewalk", road.forwardLanes._sidewalk)
                if road.backwardLanes:
                    if road.backwardLanes._sidewalk:
                        object.__setattr__(
                            crossing, "endSidewalk", road.backwardLanes._sidewalk)

        connectingRoads = tuple(connectingRoads.values())
        allRoads = roads + connectingRoads
        groups = []
        for road in allRoads:
            if road.forwardLanes:
                groups.append(road.forwardLanes)
            if road.backwardLanes:
                groups.append(road.backwardLanes)
        lanes = [lane for road in allRoads for lane in road.lanes]
        intersections = tuple(intersections.values())
        crossings = [
            crossing for road in allRoads for crossing in road.crossings]
        sidewalks, shoulders = [], []
        for group in groups:
            sidewalk = group._sidewalk
            if sidewalk:
                sidewalks.append(sidewalk)
            shoulder = group._shoulder
            if shoulder:
                shoulders.append(shoulder)

        # Add dummy maneuvers for lanes which merge/turn into another lane
        for lane in lanes:
            if not lane.maneuvers and lane._successor:
                maneuver = roadDomain.Maneuver(
                    type=roadDomain.ManeuverType.STRAIGHT,
                    startLane=lane,
                    endLane=lane._successor,
                )
                lane.maneuvers = (maneuver,)

        for crossing in crossings:
            register(crossing)

        def combine(regions):
            return PolygonalRegion.unionAll(regions, buf=self.tolerance)

        return roadDomain.Network(
            elements=allElements,
            roads=roads,
            connectingRoads=connectingRoads,
            laneGroups=tuple(groups),
            lanes=lanes,
            intersections=intersections,
            crossings=tuple(crossings),
            sidewalks=tuple(sidewalks),
            shoulders=tuple(shoulders),
            tolerance=self.tolerance,
            roadRegion=combine(roads),
            laneRegion=combine(lanes),
            intersectionRegion=combine(intersections),
            crossingRegion=combine(crossings),
            sidewalkRegion=combine(sidewalks),
        )
