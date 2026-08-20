#!/usr/bin/env python3
"""Wheel smoke checks for Medida pycolmap / pycolmap-cuda12."""

from __future__ import annotations

import argparse
import tempfile
from importlib.metadata import version
from pathlib import Path

import numpy as np
import pycolmap

SIDECAR_NAME = "medida_sparse_ext.v1.bin"


def _assert_point2d_api() -> None:
    point = pycolmap.Point2D()
    assert point.weight == 1.0, point.weight
    point.weight = 2.5
    assert point.weight == 2.5, point.weight

    assert point.constraint_point_id is None, point.constraint_point_id
    point.constraint_point_id = 7
    assert point.constraint_point_id == 7, point.constraint_point_id
    point.constraint_point_id = None
    assert point.constraint_point_id is None, point.constraint_point_id


def _assert_medida_v1_io_exists() -> None:
    reconstruction = pycolmap.Reconstruction()
    assert hasattr(reconstruction, "read_binary_medida_v1")
    assert hasattr(reconstruction, "write_binary_medida_v1")


def _assert_sidecar_roundtrip() -> None:
    empty = pycolmap.Reconstruction()
    with tempfile.TemporaryDirectory() as tmp:
        empty_dir = Path(tmp) / "empty"
        empty_dir.mkdir()
        empty.write_binary(str(empty_dir))
        assert not (empty_dir / SIDECAR_NAME).exists(), (
            "empty reconstruction must not write medida_sparse_ext.v1.bin"
        )
        loaded_empty = pycolmap.Reconstruction()
        loaded_empty.read_binary(str(empty_dir))
        assert loaded_empty.num_constraining_points3D() == 0

        recon = pycolmap.Reconstruction()
        camera = pycolmap.Camera.create_from_model_id(
            1, pycolmap.CameraModelId.PINHOLE, 500.0, 1024, 768
        )
        recon.add_camera(camera)
        constraint_id = recon.add_constraining_point3D(np.array([1.0, 2.0, 3.0]))

        sidecar_dir = Path(tmp) / "sidecar"
        sidecar_dir.mkdir()
        recon.write_binary(str(sidecar_dir))
        assert (sidecar_dir / SIDECAR_NAME).exists(), (
            "constraining points must write medida_sparse_ext.v1.bin"
        )

        loaded = pycolmap.Reconstruction()
        loaded.read_binary(str(sidecar_dir))
        assert loaded.num_constraining_points3D() == 1
        assert loaded.exists_constraining_point3D(constraint_id)
        np.testing.assert_allclose(
            loaded.constraining_point3D(constraint_id).xyz,
            [1.0, 2.0, 3.0],
        )

        medida_dir = Path(tmp) / "medida-v1"
        medida_dir.mkdir()
        recon.write_binary_medida_v1(str(medida_dir))
        assert not (medida_dir / SIDECAR_NAME).exists()
        loaded_v1 = pycolmap.Reconstruction()
        loaded_v1.read_binary_medida_v1(str(medida_dir))
        assert loaded_v1.exists_constraining_point3D(constraint_id)
        np.testing.assert_allclose(
            loaded_v1.constraining_point3D(constraint_id).xyz,
            [1.0, 2.0, 3.0],
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cuda",
        action="store_true",
        help="Also assert the CUDA 12 wheel metadata and has_cuda.",
    )
    args = parser.parse_args()

    print(pycolmap.__version__)
    _assert_point2d_api()
    _assert_medida_v1_io_exists()
    _assert_sidecar_roundtrip()

    if args.cuda:
        assert pycolmap.has_cuda, "expected CUDA-enabled pycolmap"
        print(version("pycolmap-cuda12"), pycolmap.__version__)


if __name__ == "__main__":
    main()
