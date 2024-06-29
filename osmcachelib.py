#!/usr/bin/python3

import json
import sys
import os.path
import hashlib
import requests


def get_request(url):
    fname = os.path.join("cache/", hashlib.sha256(url.encode('utf-8')).hexdigest())
    try:
        with open(fname, "r") as f:
            return f.read()
    except FileNotFoundError:
        ret = requests.get(url).content
        with open(fname, "wb") as f:
            f.write(ret)
        return ret


def get_way_data(oid):
    data = get_request(f"https://www.openstreetmap.org/api/0.6/way/{oid}.json")
    try:
        return json.loads(data)
    except:
        print(f"Unable to parse: {data}", file=sys.stderr)
        raise


def get_node_data(oid):
    data = get_request(f"https://www.openstreetmap.org/api/0.6/node/{oid}.json")
    try:
        return json.loads(data)
    except:
        print(f"Unable to parse: {data}", file=sys.stderr)
        raise
