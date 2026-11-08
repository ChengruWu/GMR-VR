"""Build an isolated Body wheel using this checkout's upstream GMR solver."""

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "packaging" / "body" / "dist"
PROJECT = '''[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "gmr-body"
version = "1.0.0"
description = "Experimental EVA Body to Gento Luna retargeting backend"
requires-python = ">=3.10,<3.14"
license = {text = "MIT"}
dependencies = ["numpy>=1.26", "scipy>=1.14", "mujoco>=3.3,<4", "mink>=1.3,<2", "qpsolvers", "daqp", "rich"]

[tool.setuptools.packages.find]
where = ["src"]
include = ["gmr_body*"]

[tool.setuptools.package-data]
gmr_body = ["resources/*.json", "resources/gento_luna/*.json", "resources/gento_luna/*.xml", "resources/gento_luna/meshes/*"]
'''


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="gmr-body-build-") as temporary:
        stage = Path(temporary)
        package = stage / "src" / "gmr_body"
        shutil.copytree(ROOT / "gmr_body", package, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        vendor = package / "_gmr"
        vendor.mkdir()
        (vendor / "__init__.py").write_text('"""Vendored GMR solver from the build checkout."""\n', encoding="utf-8")
        for name in ("motion_retarget.py", "params.py"):
            shutil.copy2(ROOT / "general_motion_retargeting" / name, vendor / name)
        shutil.copy2(ROOT / "LICENSE", stage / "LICENSE")
        (stage / "pyproject.toml").write_text(PROJECT, encoding="utf-8")
        subprocess.run([sys.executable, "-m", "build", "--no-isolation", "--wheel", "--outdir", str(OUTPUT), str(stage)], check=True)


if __name__ == "__main__":
    main()
