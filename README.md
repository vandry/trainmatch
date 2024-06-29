Experiments in matching train journeys to OSM ways
==================================================

This is extremely early experiments. I wouldn't say it works yet.

The idea is to take GPX tracks of train journeys and correlate the
positions and timestamps with train tracking data that says what
berths and signals the train is passing and when, to discover and then
annotate where those berths and signals are. But it does none of that.

The first step is to clean up the GPX by snapping it to railways as
represented in OSM. That way we remove GPS error and also when we
annotate stuff later it's probably against OSM ways and nodes that we
want to do it anyway.

Today
=====

`gpx_to_snapped_track`

Takes a sequence of points in at lat&lons (as a GPX), snaps it to
railways in OSM, and outputs a new sequence of points expressed as
our location along an OSM way instead. Each point says which OSM
way it is on (by oid), between which 2 nodes (as oids) of that way
it should be located, and how far along between those 2 nodes.

It's hella buggy and at least with my sample GPX it seems to quit
halfway through (the output track is half as long) for some reason.

`snapped_track_to_geojson`

Take the snapped track (as a binary proto) and turn it into
GeoJSON just so we can look at it.
