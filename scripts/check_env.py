"""Environment smoke test. Run after install; run again whenever something breaks.

    .venv\Scripts\python scripts\check_env.py

Exits non-zero if anything required for the geometry spine is missing.
"""

import importlib
import sys

REQUIRED = ["numpy", "scipy", "cv2", "av", "pyproj", "rasterio", "laspy", "trimesh", "open3d"]
SPINE = ["torch", "pycolmap"]
OPTIONAL = ["transformers", "ultralytics", "pymavlink", "fastapi"]


def probe(name):
    try:
        m = importlib.import_module(name)
        return True, getattr(m, "__version__", "?")
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def section(title, names, required):
    print(f"\n{title}")
    missing = []
    for n in names:
        ok, info = probe(n)
        print(f"  {'OK  ' if ok else 'FAIL'}  {n:14} {info}")
        if not ok and required:
            missing.append(n)
    return missing


def main():
    print(f"python   {sys.version.split()[0]}  ({sys.executable})")
    if sys.version_info[:2] != (3, 12):
        print("  WARNING: expected Python 3.12 -- open3d has no 3.13/3.14 wheels")

    missing = section("core (all workstreams)", REQUIRED, True)
    missing += section("geometry spine (workstream 1)", SPINE, True)
    section("optional", OPTIONAL, False)

    print("\nCUDA")
    try:
        import torch
        print(f"  torch build   {torch.__version__}")
        print(f"  cuda runtime  {torch.version.cuda}")
        avail = torch.cuda.is_available()
        print(f"  available     {avail}")
        if avail:
            print(f"  device        {torch.cuda.get_device_name(0)}")
            total = torch.cuda.get_device_properties(0).total_memory / 1024**3
            cap = torch.cuda.get_device_capability(0)
            print(f"  vram          {total:.1f} GiB   sm_{cap[0]}{cap[1]}")
            x = torch.randn(1000, 1000, device="cuda")
            print(f"  matmul        OK ({float((x @ x).sum()):.1f})")
            if total < 7:
                print("  NOTE: <7 GiB VRAM -- downscale to ~960px and tile large scenes")
        else:
            print("  FAIL: CPU-only build or driver mismatch.")
            print("        Reinstall: pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128")
            missing.append("torch-cuda")
    except Exception as e:
        print(f"  FAIL: {type(e).__name__}: {e}")
        missing.append("torch")

    if missing:
        print(f"\nFAILED -- missing: {', '.join(sorted(set(missing)))}")
        return 1
    print("\nAll good.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
