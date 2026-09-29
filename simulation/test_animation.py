"""Unit tests for the animation reference implementation (src/robot/animation.py)."""
import json

import numpy as np
import pytest
from google.protobuf import json_format

from src.platform_shared import animation_pb2 as pb


def test_proto_json_round_trip_keeps_the_leg_oneof():
    src = {
        "name": "rt",
        "schema": 1,
        "keyframes": [
            {"time": 0},
            {"time": 1.5, "ease": "EASE_IN_OUT", "body": {"roll": 0.1, "z": 20},
             "legs": [{"foot": {"z": 30}}, {"joints": {"femur": 80, "tibia": -110}},
                      {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
        ],
        "params": [{"id": "FOOT_LIFT", "min": 0.5, "defaultValue": 1, "max": 1.5}],
    }
    msg = json_format.ParseDict(src, pb.Animation())
    assert msg.keyframes[1].legs[0].WhichOneof("target") == "foot"
    assert msg.keyframes[1].legs[1].WhichOneof("target") == "joints"
    assert msg.keyframes[1].legs[2].WhichOneof("target") == "foot"
    assert msg.params[0].default_value == 1
    back = json.loads(json_format.MessageToJson(msg))
    assert back["keyframes"][1]["legs"][1] == {"joints": {"femur": 80, "tibia": -110}}
    assert pb.Animation.FromString(msg.SerializeToString()) == msg
