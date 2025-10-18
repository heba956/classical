from __future__ import annotations

import argparse
import os
import sys

# Ensure project root is on sys.path when running as a script: e.g., `python src/run.py`
_CURRENT_DIR = os.path.dirname(__file__)
_PROJECT_ROOT = os.path.abspath(os.path.join(_CURRENT_DIR, os.pardir))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from src.scenarios import get_scenario_names, make_scenario
from src.tester import PathTester


def main() -> None:
    parser = argparse.ArgumentParser(description="Run PathTester for a selected scenario.")
    parser.add_argument(
        "--scenario",
        type=str,
        default="1",
        choices=get_scenario_names(),
        help="Scenario name to load",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=3.0,
        help="Approximate track width (meters) used when only one side of cones is visible",
    )
    args = parser.parse_args()

    cones, car_pose = make_scenario(args.scenario)
    tester = PathTester(cones=cones, car_pose=car_pose, track_width=args.width)
    tester.run()


if __name__ == "__main__":
    main()


