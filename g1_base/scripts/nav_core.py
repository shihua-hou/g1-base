#!/usr/bin/env python3

from g1_base.common import route_file
from g1_base.nav_core import run_mission_from_yaml

__all__ = ["route_file", "run_mission_from_yaml"]


if __name__ == "__main__":
    run_mission_from_yaml(route_file("default.yaml"))
