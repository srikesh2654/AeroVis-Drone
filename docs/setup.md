# Environment setup

Verified against the target machine on 2026-09-13.

## Why Python 3.12 specifically

It is the only version with no gaps in the dependency set:

| package | 3.11 | 3.12 | 3.13 | 3.14 |
|---|---|---|---|---|
| open3d | yes | yes | **no** | **no** |
| rasterio / pyproj / numpy / scipy | **no** | yes | yes | yes |
| pymavlink | yes | yes | yes | **no** |
| pycolmap / opencv / laspy / av / trimesh | yes | yes | yes | yes |

Open3D caps from above, the geo stack's current releases cut off 3.11 from below.
Do not "upgrade" the venv to 3.13 or 3.14 -- Open3D will disappear.

## Install

Everyone (workstreams 2-6) -- no CUDA, no COLMAP, no compiler:

```
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\python -m pip install -r requirements-lite.txt
```

Workstream 1 (geometry spine) adds the heavy stack. **torch must come from the
CUDA index first**, or pip resolves the CPU-only build from PyPI:

```
.venv\Scripts\python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
.venv\Scripts\python -m pip install -r requirements.txt
```

Verify:

```
.venv\Scripts\python scripts\check_env.py
```

`torch.cuda.is_available()` must print `True`. If it prints `False`, the CPU
wheel got installed -- `pip uninstall torch torchvision` and redo the CUDA line.

## Machine constraints

- **RTX 3050 Laptop, 6 GB VRAM**, Ampere sm_86. Driver 616.92 / CUDA UMD 13.4.
  Downscale frames to ~960px and tile large scenes.
- **No CUDA Toolkit, no MSVC.** Nothing compiles from source. Prebuilt wheels only.
  - `gsplat` (Tier C Gaussian splatting) needs both -- confirmed cut from scope.
  - `pdal` has no Windows wheels at all -- use GDAL CLI or `rasterio`.
- `pycolmap` ships a prebuilt cp312 wheel, so **COLMAP needs no build**. The
  standalone COLMAP/OpenMVS binaries are only needed for CLI dense reconstruction.

## Fixture-first development

Nobody outside workstream 1 should need COLMAP to test their own code. Committed
fixtures under `tests/fixtures/` stand in for real pipeline output, so every
downstream module runs in seconds against `requirements-lite.txt` alone.

Run the tests:

```
.venv\Scripts\python -m pytest -q
```
