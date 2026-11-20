"""Your hand-in. We run:  python3 planner.py scenario.json plan.json

Read the scenario from argv[1], write a plan to argv[2].
Keep it within 60 seconds per scenario (a run that crashes, times out or writes an invalid plan scores 0).
"""
import os
import sys

# Ensure sibling modules in the handin directory are importable regardless of cwd
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from fleetkit.model import Scenario, load_json, save_json
from solver import solve_scenario


def main() -> None:
    scenario = Scenario.from_dict(load_json(sys.argv[1]))
    plan = solve_scenario(scenario, time_budget_s=45.0)
    save_json(plan, sys.argv[2])


if __name__ == "__main__":
    main()
