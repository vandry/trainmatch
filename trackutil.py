#!/usr/bin/python3

import datetime
import math

from osmcachelib import get_way_data, get_node_data
import track_pb2


def gcdistance(lat1, lon1, lat2, lon2):
    """Great circle angular distance between (lat1,lon1) and (lat2,lon2)."""
    lat1, lon1, lat2, lon2 = map(math.radians, (lat1, lon1, lat2, lon2))
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    sdlat = math.sin(dlat/2)
    sdlon = math.sin(dlon/2)
    a = sdlat*sdlat + math.cos(lat1) * math.cos(lat2) * sdlon*sdlon
    return 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def get_node_latlons(nodeids):
    for oid in nodeids:
        node = get_node_data(oid)
        node_element = node['elements'][0]
        yield node_element['lat'], node_element['lon']


class WayData:

    def __init__(self, way_id):
        self.way_id = way_id
        data = get_way_data(way_id)
        self.nodes = data['elements'][0]['nodes']
        self.node_latlon = list(get_node_latlons(self.nodes))

        self.node_distance_to_next = []
        for i in range(1, len(self.nodes)):
            lat0, lon0 = self.node_latlon[i-1]
            lat1, lon1 = self.node_latlon[i]
            self.node_distance_to_next.append(gcdistance(lat0, lon0, lat1, lon1))

    def point_pos_in_way(self, p: track_pb2.TrackPoint):
        if p.HasField('exact_node_id'):
            pos_v = [i for i, n in enumerate(self.nodes)
                     if n == p.exact_node_id]
            if not pos_v:
                raise RuntimeError('node for TrackPoint %r does not appear in nodes for way_id %d' % (p, self.way_id))
            if len(pos_v) > 1:
                raise RuntimeError('node for TrackPoint %r appears more than once in nodes for way_id %d' % (p, self.way_id))
            return pos_v[0], pos_v[0]
        elif p.HasField('point_along_segment'):
            pos_v = [i for i, n in enumerate(self.nodes)
                     if n in (p.point_along_segment.node_id_0, p.point_along_segment.node_id_1)]
            nodes_seen = frozenset(self.nodes[i] for i in pos_v)
            if len(nodes_seen) != 2:
                raise RuntimeError('expected 2 nodes for TrackPoint %r in way_id %d, got %d' % (p, self.way_id, len(nodes_seen)))
            if len(pos_v) != 2:
                raise RuntimeError('duplicate nodes in way_id %d?' % (self.way_id,))
            if pos_v[1] != pos_v[0] + 1:
                raise RuntimeError('expected nodes for TrackPoint %r to appear connsectively in way_id %d' % (p, self.way_id))
            return pos_v[0], pos_v[1]

    def point_distance_to_end(self, p: track_pb2.TrackPoint, direction: int):
        """Distance from p to the end of the way (direction=1) or back to start (-1)."""
        d = 0.0
        pos0, _ = self.point_pos_in_way(p)
        if p.HasField('point_along_segment'):
            # initially a fractional distance between these 2 points.
            dx = self.node_distance_to_next[pos0]
            if direction == 1:
                fraction = 1.0 - p.point_along_segment.distance
                pos0 += 1
            else:
                fraction = p.point_along_segment.distance
            d = fraction * dx
        # Plus each complete segment from here to the requested end
        if direction == 1:
            while pos0 < len(self.nodes) - 1:
                d += self.node_distance_to_next[pos0]
                pos0 += 1
        else:
            while pos0 > 0:
                pos0 -= 1
                d += self.node_distance_to_next[pos0]
        return d

    def create_point_between(self, t: datetime.datetime, p0: track_pb2.TrackPoint, p1: track_pb2.TrackPoint, direction: int):
        p0_timestamp = p0.timestamp.ToDatetime()
        dt_points = p1.timestamp.ToDatetime() - p0_timestamp
        dt_desired = t - p0_timestamp
        frac_desired = dt_desired / dt_points
        p0_to_end = self.point_distance_to_end(p0, -1)
        p1_to_end = self.point_distance_to_end(p1, -1)
        dx = p1_to_end - p0_to_end
        if dx >= 0.0:
            desired_distance = p0_to_end + dx * frac_desired
        else:
            desired_distance = p1_to_end - dx * frac_desired

        d_acc = 0.0
        prev_node_id = self.nodes[0]
        for node_id, distance_to_here in zip(self.nodes[1:], self.node_distance_to_next):
            if d_acc + distance_to_here <= desired_distance:
                d_acc += distance_to_here
                if d_acc == desired_distance:  # exact hit
                    p = track_pb2.TrackPoint(way_id=self.way_id, exact_node_id=node_id)
                    p.timestamp.FromDatetime(t)
                    return p
                prev_node_id = node_id
                continue
            left_to_cover = desired_distance - d_acc
            frac = left_to_cover / distance_to_here
            assert 0.0 < frac < 1.0

            p = track_pb2.TrackPoint(way_id=self.way_id)
            pas = p.point_along_segment
            pas.node_id_0 = prev_node_id
            pas.node_id_1 = node_id
            pas.distance = frac
            p.timestamp.FromDatetime(t)
            return p

        raise RuntimeError('overran bounds, desired_distance too large')


class WayFollower:

    def __init__(self, way: WayData, direction: int):
        self.way = way
        self.direction = direction
        self.points = []
        self.t0 = self.t1 = None

    def fit_timestamp(self, t: datetime.datetime):
        # Quick reject attempts later than we handle
        if self.t1 is None:
            if t > self.points[-1].timestamp.ToDatetime():
                return None
        elif t > self.t1:
            return None

        # Establish the first point or create a synthetic first point.
        if self.t0 is None:
            p_prev = self.points[0]
            if t < p_prev.timestamp.ToDatetime():  # quick reject
                return None
        else:
            if t < self.t0:  # quick reject
                return None
            # synthetic point
            t0_node_id = self.way.nodes[0 if self.direction == 1 else -1]
            p_prev = track_pb2.TrackPoint(way_id=self.way.way_id, exact_node_id=t0_node_id)
            p_prev.timestamp.FromDatetime(self.t0)
            if t == self.t0:
                return p_prev

        for p in self.points:
            p_t = p.timestamp.ToDatetime()
            if t == p_t:
                return p
            if p is p_prev:
                continue  # happens if self.t0 is None above. We need one more point.
            if t > p_t:
                continue  # keep searching
            return self.way.create_point_between(t, p_prev, p, self.direction)

        # after last point
        end_node_id = self.way.nodes[-1 if self.direction == 1 else 0]
        p_end = track_pb2.TrackPoint(way_id=self.way.way_id, exact_node_id=end_node_id)
        p_end.timestamp.FromDatetime(self.t1)
        if t == self.t1:
            return p_end
        return self.way.create_point_between(t, self.points[-1], p_end, self.direction)


class TrackFollower:

    def __init__(self, t: track_pb2.Track, get_way):
        p0 = t.p[0]
        way0 = get_way(p0.way_id)
        p0_pos = way0.point_pos_in_way(p0)
        direction = 0
        for p in t.p[1:]:
            way1 = get_way(p.way_id)
            if way1.way_id == way0.way_id:
                p1_pos = way0.point_pos_in_way(p)
                if p1_pos == p0_pos:
                    continue
                direction = 1 if p1_pos > p0_pos else -1
                break
            elif way0.nodes[0] in way1.nodes:
                # The first node of the first way matches up with the next way.
                # We are going backwards.
                direction = -1
                break
            elif way0.nodes[-1] in way1.nodes:
                # The last node of the first way matches up with the next way.
                # We are going forwards.
                direction = 1
                break
            else:
                raise RuntimeError('ways %d and %d are unconnected' % (way0.way_id, way1.way_id))
        if direction == 0:
            raise RuntimeError('cannot determine travel direction at point %r' % (p,))

        f = WayFollower(way0, direction)
        f.points.append(p0)
        self.ways = [f]

        for p in t.p[1:]:
            way1 = get_way(p.way_id)
            if way1.way_id == f.way.way_id:
                 p0_pos = f.way.point_pos_in_way(f.points[-1])
                 p1_pos = f.way.point_pos_in_way(p)
                 if p0_pos != p1_pos:
                     direction = 1 if p1_pos > p0_pos else -1
                     if direction != f.direction:
                         raise RuntimeError('direction not monotonic: established direction %d, direction from %r to %r is %d' % (f.direction, f.points[-1], p, direction))
                 f.points.append(p)
                 continue
            # New way
            if f.direction == 1:
                old_way_last_node = f.way.nodes[-1]
            else:
                old_way_last_node = f.way.nodes[0]
            if old_way_last_node == way1.nodes[0]:
                # The last node of the old way appears at the beginning of the new way.
                direction = 1
            elif old_way_last_node == way1.nodes[-1]:
                # The last node of the old way appears at the end of the new way.
                direction = -1
            else:
                raise RuntimeError('way %d is not connected end to end with the %s of way %d' % (way1.way_id, ('end' if f.direction == 1 else 'beginning'), f.way.way_id))
            f = WayFollower(way1, direction)
            f.points.append(p)
            self.ways.append(f)

        # Compute the time we joined and left each way.
        for i in range(1, len(self.ways)):
            f0 = self.ways[i-1]
            f1 = self.ways[i]
            t0 = f0.points[-1].timestamp.ToDatetime()
            dt = f1.points[0].timestamp.ToDatetime() - t0
            dx0 = f0.way.point_distance_to_end(f0.points[-1], f0.direction)
            dx1 = f1.way.point_distance_to_end(f1.points[0], -f1.direction)
            dx = dx0 + dx1
            # Interpolate constant velocity between the 2 points.
            t = t0 + dt * (dx0 / dx)
            f0.t1 = t
            f1.t0 = t

    def fit_timestamp(self, t: datetime.datetime):
        if t < self.ways[0].points[0].timestamp.ToDatetime():
            return None
        if t > self.ways[-1].points[-1].timestamp.ToDatetime():
            return None
        for wf in self.ways:
            point = wf.fit_timestamp(t)
            if point is not None:
                return point


class TrackSequenceFollower:

    def __init__(self, ts: track_pb2.TrackSequence):
        self.way_index = {}
        self.tracks = []
        for track_pb in ts.t:
            self.tracks.append(TrackFollower(track_pb, self._get_way))

    def _get_way(self, way_id):
        try:
            return self.way_index[way_id]
        except KeyError:
            w = WayData(way_id)
            self.way_index[way_id] = w
            return w

    def fit_timestamp(self, t: datetime.datetime):
        """Generate a synthetic point along the track between the points just before and after t."""
        for tf in self.tracks:
            point = tf.fit_timestamp(t)
            if point is not None:
                return point
