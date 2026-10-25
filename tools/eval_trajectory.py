#!/usr/bin/env python3
"""Score a GLIM trajectory against a reference: ATE after rigid alignment, as JSON.

  tools/eval_trajectory.py <session>/dump/traj_imu.txt <session>/reference.tum [max_dt_s]

Sessions with an `evaluation:` block get this automatically at session end (evaluation.json);
this is the same computation for any pair of TUM files. Pure Python, no dependencies.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "supervisor"))
from glim_supervisor import trajectory  # noqa: E402

if __name__ == "__main__":
    try:
        raise SystemExit(trajectory.main(sys.argv[1:]))
    except (OSError, ValueError) as error:
        sys.exit(f"eval_trajectory: {error}")
