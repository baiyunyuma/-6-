#!/usr/bin/env python3
"""Identify rigid-body inertial + friction parameters from motor_driver CSV.

Method
------
Pinocchio builds the joint-torque regressor Y(q, v, a) from the URDF kinematics.
The link inertial parameters (mass / first moment / inertia) are the unknowns,
so the URDF mass values are only a prior -- the data identifies the real ones.

For every sample we solve:

    tau = Y(q, v, a) * pi  +  offset_i + viscous_i*dq_i + coulomb_i*tanh(s*dq_i)

The per-joint offset absorbs motor torque bias / static friction.  The solution
is ridge-regularized toward the URDF inertial parameters (small lambda).

Usage
-----
  python3 identify_inertial.py /tmp/zfy_ident_*.csv \
      --urdf ~/zfy_arm/zfy_arm_control/source/robot.urdf \
      --mode full --ridge 1e-4 --max-samples 6000

The identified `pi` vector can be used directly for gravity:
    tau_g(q) = Y(q, 0, 0) * pi[:60]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

NJ = 6
NP_INERTIA = 10 * NJ          # 60
NP_FRICTION = 3 * NJ          # offset, viscous, coulomb


def load_csv(path: str) -> dict:
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise RuntimeError("empty csv: " + path)

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=np.float64)

    q = np.column_stack([col("q_j%d" % i) for i in range(1, NJ + 1)])
    dq = np.column_stack([col("dq_j%d" % i) for i in range(1, NJ + 1)])
    tau = np.column_stack([col("tau_j%d" % i) for i in range(1, NJ + 1)])
    qd = np.column_stack([col("q_des_j%d" % i) for i in range(1, NJ + 1)])
    t = col("t_mono")
    enabled = col("enabled") if "enabled" in rows[0] else np.ones(len(rows))
    sending = col("sending") if "sending" in rows[0] else np.ones(len(rows))
    finite = (np.isfinite(q).all(1) & np.isfinite(dq).all(1)
              & np.isfinite(tau).all(1) & np.isfinite(t))
    mask = (enabled > 0.5) & (sending > 0.5) & finite
    return dict(path=path, t=t, q=q, dq=dq, tau=tau, qd=qd, mask=mask)


def smooth(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    k = np.ones(n, dtype=np.float64) / n
    out = np.empty_like(x)
    for j in range(x.shape[1]):
        out[:, j] = np.convolve(x[:, j], k, mode="same")
    return out


def regressor(model, data, q, v, a) -> np.ndarray:
    return np.array(
        pin.computeJointTorqueRegressor(
            model, data, np.asarray(q), np.asarray(v), np.asarray(a)
        ),
        dtype=np.float64,
    )


def prior_parameters(model) -> np.ndarray:
    x0 = np.zeros(NP_INERTIA)
    for j in range(1, model.njoints):
        x0[10 * (j - 1): 10 * j] = np.array(
            model.inertias[j].toDynamicParameters(), dtype=np.float64
        )
    return x0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument(
        "--urdf",
        default="/home/sky/zfy_arm/zfy_arm_control/source/robot.urdf",
    )
    ap.add_argument("--mode", choices=["full", "static"], default="full")
    ap.add_argument("--ridge", type=float, default=1e-4)
    ap.add_argument("--max-samples", type=int, default=6000)
    ap.add_argument("--smooth", type=int, default=5)
    ap.add_argument("--static-dq", type=float, default=0.05)
    ap.add_argument("--static-qerr", type=float, default=0.20)
    ap.add_argument("--friction-slope", type=float, default=100.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    if model.nq != NJ or model.nv != NJ:
        raise RuntimeError("expected nq=nv=6, got %d/%d" % (model.nq, model.nv))
    x0 = np.concatenate([prior_parameters(model), np.zeros(NP_FRICTION)])

    datasets = [load_csv(p) for p in args.csv]
    rows_A, rows_b, used = [], [], 0
    for d in datasets:
        t = d["t"]
        dq = smooth(d["dq"], args.smooth)
        ddq = np.gradient(dq, t, axis=0) if len(t) > 2 else np.zeros_like(dq)
        ddq = smooth(ddq, args.smooth)
        qerr = np.max(np.abs(d["q"] - d["qd"]), axis=1)
        dqmax = np.max(np.abs(dq), axis=1)
        if args.mode == "static":
            ok = d["mask"] & (dqmax <= args.static_dq) & (qerr <= args.static_qerr)
        else:
            ok = d["mask"] & (qerr <= args.static_qerr)
        idx = np.nonzero(ok)[0]
        if len(idx) > args.max_samples:
            idx = idx[np.linspace(0, len(idx) - 1, args.max_samples).astype(int)]
        print("%s: %d samples (%s)"
              % (Path(d["path"]).name, len(idx), args.mode))
        used += len(idx)
        for k in idx:
            q = d["q"][k]
            v = dq[k]
            a = ddq[k]
            Y = regressor(model, data, q, v, a) if args.mode == "full" \
                else regressor(model, data, q, np.zeros(NJ), np.zeros(NJ))
            F = np.zeros((NJ, NP_FRICTION))
            for i in range(NJ):
                F[i, 3 * i] = 1.0                      # constant / bias
                F[i, 3 * i + 1] = v[i]                 # viscous
                F[i, 3 * i + 2] = np.tanh(args.friction_slope * v[i])
            rows_A.append(np.hstack([Y, F]))
            rows_b.append(d["tau"][k])

    if not rows_A:
        raise RuntimeError("no usable samples")
    A = np.vstack(rows_A)
    b = np.concatenate(rows_b)
    print("regression: A=%s b=%s" % (A.shape, b.shape))

    # ridge toward the URDF prior
    lam = args.ridge
    AtA = A.T @ A + lam * np.eye(A.shape[1])
    Atb = A.T @ b + lam * x0
    try:
        x = np.linalg.solve(AtA, Atb)
    except np.linalg.LinAlgError:
        x = np.linalg.lstsq(AtA, Atb, rcond=None)[0]

    pred = A @ x
    err = (pred - b).reshape(-1, NJ)
    rmse = np.sqrt(np.mean(err * err, axis=0))
    print("fit RMSE (N*m):", " ".join("%.4f" % z for z in rmse))

    pi = x[:NP_INERTIA]
    links = []
    for j in range(1, model.njoints):
        dyn = pi[10 * (j - 1): 10 * j]
        try:
            inert = pin.Inertia.FromDynamicParameters(dyn)
            model.inertias[j] = inert
            mass, lever = float(inert.mass), np.array(inert.lever)
            inertia = np.array(inert.inertia)
        except Exception as exc:
            print("  %s: cannot build Inertia: %s" % (model.names[j], exc))
            mass, lever, inertia = float(dyn[0]), np.zeros(3), np.zeros((3, 3))
        com = lever if abs(mass) > 1e-9 else np.array(dyn[1:4])
        links.append(dict(joint=model.names[j], mass=mass,
                          com=[float(z) for z in com],
                          inertia=[float(z) for z in inertia.reshape(-1)]))
        print("  %-14s mass=%+9.4f kg  com=[%+.4f %+.4f %+.4f]"
              % (model.names[j], mass, com[0], com[1], com[2]))

    friction = {}
    for i in range(NJ):
        off, vis, cou = (float(x[NP_INERTIA + 3 * i]),
                         float(x[NP_INERTIA + 3 * i + 1]),
                         float(x[NP_INERTIA + 3 * i + 2]))
        friction["joint%d" % (i + 1)] = dict(offset=off, viscous=vis, coulomb=cou)
        print("  joint%d offset=%+.4f viscous=%+.4f coulomb=%+.4f"
              % (i + 1, off, vis, cou))

    out = dict(
        urdf=os.path.abspath(args.urdf),
        csv=[os.path.abspath(p) for p in args.csv],
        mode=args.mode,
        samples=int(used),
        fit_rmse=[float(z) for z in rmse],
        links=links,
        friction=friction,
        pi=[float(z) for z in pi],
        x=[float(z) for z in x],
        note=("pi: 6 links x [m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz];"
              " friction: 6 x [offset, viscous, coulomb]"),
    )
    out_path = args.out or str(Path(args.csv[0]).with_suffix(".inertial.json"))
    with open(out_path, "w") as fh:
        json.dump(out, fh, indent=2)
    print("wrote:", out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
