#!/usr/bin/python3

import bisect
import datetime
import math

from osmcachelib import get_way_data, get_node_data, get_node_way_data
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


class WayFollower:

    def __init__(self, way: WayData, direction: int):
        self.way = way
        self.direction = direction


class TrackFollowerCursor(tuple):
    """Marks a precise position along a Track,with methods for moving forward."""

    def __new__(self, tf, wfi: int, ni: int, frac_after_node: float, odometer: float):
        return tuple.__new__(TrackFollowerCursor, (tf, wfi, ni, frac_after_node, odometer))

    def odometer(self):
        return self[4]

    def _travel(self, get_want_frac):
        tf, wfi, ni, frac_after_node, odometer = self
        wf = tf.ways[wfi]
        while True:
            want_frac = get_want_frac(wf, ni, frac_after_node, odometer)
            if want_frac is not None:
                break  # We have reached the correct node
            next_way = False
            if wf.direction == 1:
                odometer += wf.way.node_distance_to_next[ni] * (1.0 - frac_after_node)
                frac_after_node = 0.0
                ni += 1
                if ni == len(wf.way.node_distance_to_next):
                    next_way = True
            else:
                odometer += wf.way.node_distance_to_next[ni] * frac_after_node
                frac_after_node = 1.0
                ni -= 1
                if ni == -1:
                    next_way = True
            if next_way:
                wfi += 1
                if wfi >= len(tf.ways):
                    return None
                wf = tf.ways[wfi]
                if wf.direction == 1:
                    ni = 0
                    frac_after_node = 0.0
                else:
                    ni = len(wf.way.nodes) - 2
                    frac_after_node = 1.0
        to_move = want_frac - frac_after_node
        if (to_move < 0.0) if wf.direction == 1 else (to_move > 0.0):
            raise RuntimeError('TrackFollowerCursor already past the desired point')
        frac_after_node += to_move
        odometer += math.fabs(to_move) * wf.way.node_distance_to_next[ni]
        return type(self)(tf, wfi, ni, frac_after_node, odometer)

    def travel_to_point(self, p: track_pb2.TrackPoint):
        """Travel forward to the position of TrackPoint p, return new cursor."""
        if p.HasField('exact_node_id'):
            want_node0, want_node1, want_frac = p.exact_node_id, p.exact_node_id, 0.0
            def get_want_frac(wf: WayFollower, ni: int, frac_after_node: float, odometer: float):
                if wf.way.way_id != p.way_id:
                    return None
                if wf.way.nodes[ni] == p.exact_node_id:
                    return 0.0
                if wf.way.nodes[ni + 1] == p.exact_node_id:
                    return 1.0
                return None
        elif p.HasField('point_along_segment'):
            def get_want_frac(wf: WayFollower, ni: int, frac_after_node: float, odometer: float):
                if wf.way.way_id != p.way_id:
                    return None
                if wf.way.nodes[ni] == p.point_along_segment.node_id_0:
                    return p.point_along_segment.distance
                return None
        else:
            raise RuntimeError('no point in TrackPoint')
        return self._travel(get_want_frac)

    def travel_to_odometer(self, desired_odometer: float):
        """Travel forward until the desired cumulative travel distance, return new cursor."""
        def get_want_frac(wf: WayFollower, ni: int, frac_after_node: float, odometer: float):
            this_interval_distance = wf.way.node_distance_to_next[ni]
            if wf.direction == 1:
                odometer_at_node0 = odometer - this_interval_distance * frac_after_node
                want_frac = (desired_odometer - odometer_at_node0) / this_interval_distance
                if want_frac > 1.0:
                    return None
            else:
                odometer_at_node1 = odometer - this_interval_distance * (1.0 - frac_after_node)
                want_frac = 1.0 - ((desired_odometer - odometer_at_node1) / this_interval_distance)
                if want_frac < 0.0:
                    return None
            return want_frac
        return self._travel(get_want_frac)

    def set_point(self, p: track_pb2.TrackPoint):
        """Override the position in TrackPoint p to that of this cursor."""
        tf, wfi, ni, frac_after_node, _ = self
        wf = tf.ways[wfi]
        p.way_id = wf.way.way_id
        if frac_after_node == 0.0:
            p.exact_node_id = wf.way.nodes[ni]
        elif frac_after_node == 1.0:
            p.exact_node_id = wf.way.nodes[ni + 1]
        else:
            pas = p.point_along_segment
            pas.node_id_0 = wf.way.nodes[ni]
            pas.node_id_1 = wf.way.nodes[ni + 1]
            pas.distance = frac_after_node


def _search_for_way(seen: set, get_way, here: WayFollower, want_way: WayData, depth: int):
    seen.add(here.way.way_id)
    if here.direction == 1:
        old_way_last_node = here.way.nodes[-1]
    else:
        old_way_last_node = here.way.nodes[0]
    if old_way_last_node == want_way.nodes[0]:
        # The last node of the old way appears at the beginning of the new way.
        return [WayFollower(want_way, 1)]
    elif old_way_last_node == want_way.nodes[-1]:
        # The last node of the old way appears at the end of the new way.
        return [WayFollower(want_way, -1)]
    if depth == 0:
        return []

    membership_data = get_node_way_data(old_way_last_node)
    for element in membership_data['elements']:
        if element['type'] != 'way':
            continue
        next_way_id = element['id']
        if next_way_id in seen:
            continue
        next_way = get_way(next_way_id)
        if old_way_last_node == next_way.nodes[0]:
            next_wf = WayFollower(next_way, 1)
        elif old_way_last_node == next_way.nodes[-1]:
            next_wf = WayFollower(next_way, -1)
        else:
            #raise RuntimeError('way %d is not connected end to end with the %s of way %d' % (next_way_id, ('end' if here.direction == 1 else 'beginning'), here.way.way_id))
            continue
        path = _search_for_way(seen, get_way, next_wf, want_way, depth - 1)
        if path:
            return [next_wf] + path
    return []


class TrackFollower:

    def __init__(self, t: track_pb2.Track, get_way):
        self.track = t
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
        self.ways = [f]
        self.point_index_to_way_index = [0]
        prev_p = t.p[0]

        for p in t.p[1:]:
            way1 = get_way(p.way_id)
            if way1.way_id == f.way.way_id:
                 p0_pos = f.way.point_pos_in_way(prev_p)
                 p1_pos = f.way.point_pos_in_way(p)
                 if p0_pos != p1_pos:
                     direction = 1 if p1_pos > p0_pos else -1
                     if direction != f.direction:
                         raise RuntimeError('direction not monotonic: established direction %d, direction from %r to %r is %d' % (f.direction, f.points[-1], p, direction))
                 self.point_index_to_way_index.append(len(self.ways) - 1)
                 prev_p = p
                 continue
            # New way
            path = _search_for_way(set(), get_way, f, way1, 10)
            if not path:
                raise RuntimeError('unable to find a path from the %s of way %d to way %d' % (('end' if f.direction == 1 else 'beginning'), f.way.way_id, way1.way_id))
            f = path[-1]
            self.ways.extend(path)
            self.point_index_to_way_index.append(len(self.ways) - 1)
            prev_p = p

    def fit_timestamp(self, t: datetime.datetime):
        if t < self.track.p[0].timestamp.ToDatetime():
            return None
        if t > self.track.p[-1].timestamp.ToDatetime():
            return None
        a = [p.timestamp.ToDatetime() for p in self.track.p]
        i = bisect.bisect(a, t)
        p_before = self.track.p[i-1]
        if t == p_before.timestamp.ToDatetime():
            p = track_pb2.TrackPoint()
            p.CopyFrom(p_before)
            return p
        p_after = self.track.p[i]
        wfi = self.point_index_to_way_index[i-1]
        wf = self.ways[wfi]
        if wf.direction == 1:
            cursor = TrackFollowerCursor(self, wfi, 0, 0.0, 0.0)
        else:
            cursor = TrackFollowerCursor(self, wfi, len(wf.way.nodes) - 2, 1.0, 0.0)
        cursor_before = cursor.travel_to_point(p_before)
        cursor_after = cursor_before.travel_to_point(p_after)

        # Interpolate assuming constant velocity.
        dx = cursor_after.odometer() - cursor_before.odometer()
        dt = p_after.timestamp.ToDatetime() - p_before.timestamp.ToDatetime()
        want_dt = t - p_before.timestamp.ToDatetime()
        want_dx = dx * (want_dt / dt)
        cursor = cursor_before.travel_to_odometer(cursor_before.odometer() + want_dx)

        p = track_pb2.TrackPoint()
        p.timestamp.FromDatetime(t)
        cursor.set_point(p)
        return p

    def begin(self):
        """Return a TrackFollowerCursor located at the track of this track."""
        if self.ways[0].direction == 1:
            return TrackFollowerCursor(self, 0, 0, 0.0, 0.0)
        else:
            return TrackFollowerCursor(self, 0, len(self.ways[0].way.nodes) - 2, 1.0, 0.0)


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
