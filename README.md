# AeroVis

Georeferenced, metrically accurate 3D reconstruction from a **single-pass** drone video.

SIH problem statement: generate textured 3D terrain, structures, roads and
vegetation from one flight path -- no grid pattern, no repeat passes, no
extensive Ground Control Points.

## Status

M1 in progress. Environment and the S0 ingest contract are up; the geometry
spine is not yet wired.

## Layout

```
aerovis/
  core/      schema.py -- FlightTrack, the S0 -> downstream contract
  ingest/    S0  telemetry parsers (DJI SRT done; MAVLink, CSV, EXIF pending)
  curate/    S1  frame selection, sharpness, deblur
  masking/   S2  dynamic object masking
  sfm/       S3  pose estimation + georeferencing
  dense/     S4  dense reconstruction (Tier A preview, Tier B mesh)
  products/  S5  DSM, orthomosaic, semantics
  api/       S6  FastAPI job API
web/         S6  CesiumJS viewer
scripts/     check_env.py and other operational scripts
tests/       pytest suite + committed fixtures
```

## Getting started

See [docs/setup.md](docs/setup.md). Short version:

```
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -r requirements-lite.txt
.venv\Scripts\python -m pytest -q
```

Python 3.12 is required, not merely recommended -- see the table in the setup doc.
# AeroVis-Drone_SIH
