#!/usr/bin/env python3
"""Small EigenPy example for estimator-side linear algebra experiments."""

from __future__ import annotations

import sys
from pathlib import Path


def prefer_local_eigenpy_build() -> None:
    """Prefer the eigenpy module built inside this repository, when present."""
    repo_root = Path(__file__).resolve().parents[2]
    build_python = repo_root / "eigenpy" / "build" / "python"
    if build_python.exists():
        sys.path.insert(0, str(build_python))


def format_vector(values, precision: int = 6) -> str:
    return "[" + ", ".join(f"{value:.{precision}g}" for value in values) + "]"


def main() -> int:
    prefer_local_eigenpy_build()

    try:
        import eigenpy
        import numpy as np
    except ImportError as exc:
        missing_name = exc.name or "module"
        raise SystemExit(
            f"Missing {missing_name}. Build eigenpy or run this example in an "
            "environment where eigenpy and numpy are installed."
        ) from exc

    sensor_samples = np.array(
        [
            [0.10, 0.03, -0.010],
            [0.12, 0.04, -0.020],
            [0.09, 0.02, 0.000],
            [0.13, 0.05, -0.015],
        ],
        dtype=float,
    )

    centered = sensor_samples - sensor_samples.mean(axis=0)
    covariance = centered.T @ centered / (sensor_samples.shape[0] - 1)
    covariance += np.eye(covariance.shape[0]) * 1e-6

    eigensolver = eigenpy.SelfAdjointEigenSolver(covariance)
    if eigensolver.info() != eigenpy.ComputationInfo.Success:
        raise RuntimeError(f"Eigen decomposition failed: {eigensolver.info()}")

    eigenvalues = eigensolver.eigenvalues()
    eigenvectors = eigensolver.eigenvectors()
    principal_axis = eigenvectors[:, int(np.argmax(eigenvalues))]

    llt = eigenpy.LLT(covariance)
    if llt.info() != eigenpy.ComputationInfo.Success:
        raise RuntimeError(f"LLT decomposition failed: {llt.info()}")

    unit_response = llt.solve(np.ones(covariance.shape[0], dtype=float))

    print(f"eigenpy version: {getattr(eigenpy, '__version__', 'unknown')}")
    print("sensor covariance:")
    print(covariance)
    print(f"eigenvalues: {format_vector(eigenvalues)}")
    print(f"principal axis: {format_vector(principal_axis)}")
    print(f"LLT solve result: {format_vector(unit_response)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
