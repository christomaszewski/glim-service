"""Score a GLIM trajectory against a reference — typically the INS's fused odometry.

Both are TUM files (`t x y z qx qy qz qw`). Poses are associated by nearest timestamp, the
estimate is rigidly aligned to the reference (Horn's closed-form absolute orientation: the two
trajectories live in different world frames), and the translation residuals give the absolute
trajectory error. Pure Python on purpose: it runs in the image at session end and on any laptop.

What it measures: GLIM's IMU-frame positions against the reference body's positions. If those
bodies differ (an INS reporting base_link), the lever arm between them adds error that grows with
rotation — small for a co-located IMU, and constant for a given vehicle and route, so comparisons
between tunings on one recording stay fair.
"""
import bisect
import json
import math
from pathlib import Path


def load_tum(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        fields = line.split()
        if len(fields) == 8 and not line.lstrip().startswith("#"):
            try:
                rows.append(tuple(float(v) for v in fields))
            except ValueError:
                continue
    rows.sort()
    return rows


def associate(estimate, reference, max_dt):
    """-> [(est xyz, ref xyz, t)]: each estimate pose with the reference pose nearest in time."""
    times = [row[0] for row in reference]
    pairs = []
    for row in estimate:
        i = bisect.bisect_left(times, row[0])
        best = min((j for j in (i - 1, i) if 0 <= j < len(times)),
                   key=lambda j: abs(times[j] - row[0]), default=None)
        if best is not None and abs(times[best] - row[0]) <= max_dt:
            pairs.append((row[1:4], reference[best][1:4], row[0]))
    return pairs


def _jacobi_eigen(matrix, sweeps=50):
    """Eigen-decomposition of a small symmetric matrix: (eigenvalues, eigenvectors as columns)."""
    n = len(matrix)
    a = [list(row) for row in matrix]
    v = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for _ in range(sweeps):
        off = sum(a[i][j] ** 2 for i in range(n) for j in range(n) if i != j)
        if off < 1e-22:
            break
        for p in range(n):
            for q in range(p + 1, n):
                if abs(a[p][q]) < 1e-300:
                    continue
                theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
                t = (1.0 if theta >= 0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c
                for k in range(n):
                    akp, akq = a[k][p], a[k][q]
                    a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
                for k in range(n):
                    apk, aqk = a[p][k], a[q][k]
                    a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
                for k in range(n):
                    vkp, vkq = v[k][p], v[k][q]
                    v[k][p], v[k][q] = c * vkp - s * vkq, s * vkp + c * vkq
    return [a[i][i] for i in range(n)], v


def align(source, target):
    """Rigid (R, t) minimizing sum |R s + t - d|^2 (Horn 1987, unit quaternions)."""
    n = len(source)
    cs = [sum(p[k] for p in source) / n for k in range(3)]
    cd = [sum(p[k] for p in target) / n for k in range(3)]
    m = [[0.0] * 3 for _ in range(3)]
    for s, d in zip(source, target):
        for i in range(3):
            for j in range(3):
                m[i][j] += (s[i] - cs[i]) * (d[j] - cd[j])
    sxx, sxy, sxz = m[0]
    syx, syy, syz = m[1]
    szx, szy, szz = m[2]
    big_n = [[sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
             [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
             [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
             [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz]]
    values, vectors = _jacobi_eigen(big_n)
    k = max(range(4), key=lambda i: values[i])
    w, x, y, z = (vectors[r][k] for r in range(4))
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    t = [cd[i] - sum(r[i][j] * cs[j] for j in range(3)) for i in range(3)]
    return r, t


def evaluate(estimate_path, reference_path, max_dt=0.05):
    """-> the evaluation document (raises ValueError when the two cannot be compared)."""
    estimate, reference = load_tum(estimate_path), load_tum(reference_path)
    pairs = associate(estimate, reference, max_dt)
    if len(pairs) < 3:
        raise ValueError(f"only {len(pairs)} poses within {max_dt}s of a reference pose "
                         f"({len(estimate)} estimated, {len(reference)} reference): are both on "
                         "one clock?")
    est = [p[0] for p in pairs]
    ref = [p[1] for p in pairs]
    rotation, translation = align(est, ref)
    errors = []
    for e, d in zip(est, ref):
        moved = [sum(rotation[i][j] * e[j] for j in range(3)) + translation[i] for i in range(3)]
        errors.append(math.dist(moved, d))
    length = sum(math.dist(ref[i], ref[i + 1]) for i in range(len(ref) - 1))
    return {
        "version": 1,
        "estimate": str(estimate_path), "reference": str(reference_path),
        "matched": len(pairs), "estimated": len(estimate), "reference_poses": len(reference),
        "duration_s": pairs[-1][2] - pairs[0][2],
        "reference_path_length_m": length,
        "ate_rmse_m": math.sqrt(sum(e * e for e in errors) / len(errors)),
        "ate_mean_m": sum(errors) / len(errors),
        "ate_max_m": max(errors),
        "final_error_m": errors[-1],
        "final_error_pct_of_path": 100.0 * errors[-1] / length if length > 0 else None,
        "alignment": {"rotation": rotation, "translation": translation},
    }


def main(argv):
    """python3 trajectory.py <estimate.tum> <reference.tum> [max_dt_s] -> JSON on stdout."""
    if len(argv) not in (2, 3):
        raise SystemExit("usage: eval_trajectory.py <estimate.tum> <reference.tum> [max_dt_s]")
    result = evaluate(argv[0], argv[1], float(argv[2]) if len(argv) == 3 else 0.05)
    print(json.dumps(result, indent=2))
    return 0
