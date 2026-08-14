from g1_base.common import route_file
from g1_base.nav_core import run_mission_from_yaml


def main():
    run_mission_from_yaml(route_file("default.yaml"))


if __name__ == "__main__":
    main()
