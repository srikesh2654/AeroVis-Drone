"""Canonical telemetry schema.

Every S0 ingest parser, whatever the source format, returns a `FlightTrack`.
Downstream stages (S1-S6) depend only on this module, never on a parser.
"""

from __future__ import annotations

import bisect
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path


@dataclass
class CameraIntrinsics:
    """Pinhole intrinsics in pixels. A `None` field means "unknown, must be estimated"."""

    width: int
    height: int
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None
    # Brown-Conrady (k1, k2, p1, p2, k3). None means undistorted or unknown.
    distortion: tuple[float, ...] | None = None
    # Provenance, so S3 knows how much to trust these: "exif" | "srt" | "assumed" | "selfcal"
    source: str = "assumed"

    @property
    def is_complete(self) -> bool:
        return None not in (self.fx, self.fy, self.cx, self.cy)

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_deg: float,
                  source: str = "assumed") -> CameraIntrinsics:
        """Fallback when only a horizontal field of view is known.

        Most consumer drones sit near 84 deg HFOV. A rough guess keeps the
        pipeline running on uncalibrated footage instead of refusing to start;
        S3 refines it during self-calibration.
        """
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width=width, height=height, fx=fx, fy=fx,
                   cx=width / 2.0, cy=height / 2.0, source=source)


@dataclass
class TelemetrySample:
    """One telemetry reading, timestamped relative to the first video frame."""

    t: float                              # seconds since video start
    lat: float | None = None              # WGS84 degrees
    lon: float | None = None              # WGS84 degrees
    alt_abs: float | None = None          # metres, MSL or ellipsoid as reported
    alt_rel: float | None = None          # metres above takeoff
    gimbal_yaw: float | None = None       # degrees
    gimbal_pitch: float | None = None     # degrees, negative = looking down
    gimbal_roll: float | None = None
    drone_yaw: float | None = None
    drone_pitch: float | None = None
    drone_roll: float | None = None
    focal_len_mm: float | None = None
    gps_accuracy: float | None = None     # metres, if reported

    @property
    def has_position(self) -> bool:
        return self.lat is not None and self.lon is not None


@dataclass
class FlightTrack:
    """Normalized telemetry for one video.

    The single contract between S0 and everything downstream.
    """

    video_path: str
    samples: list[TelemetrySample] = field(default_factory=list)
    intrinsics: CameraIntrinsics | None = None
    fps: float | None = None
    frame_count: int | None = None
    # "dji_srt" | "mavlink" | "csv" | "exif" | "none"
    source_format: str = "unknown"
    warnings: list[str] = field(default_factory=list)

    # -- quality gates -------------------------------------------------

    @property
    def has_geo(self) -> bool:
        """True if georeferencing (S3) is possible at all."""
        return sum(1 for s in self.samples if s.has_position) >= 2

    @property
    def duration(self) -> float:
        return self.samples[-1].t - self.samples[0].t if len(self.samples) >= 2 else 0.0

    def degraded_reasons(self) -> list[str]:
        """Why this track yields a lower-quality reconstruction. Surfaced in the quality report."""
        reasons = []
        if not self.samples:
            reasons.append("no telemetry: reconstruction will be scale-free")
        elif not self.has_geo:
            reasons.append("no GPS positions: output cannot be georeferenced")
        if self.intrinsics is None or not self.intrinsics.is_complete:
            reasons.append("incomplete intrinsics: S3 must self-calibrate")
        if self.samples and self.duration > 0:
            rate = len(self.samples) / self.duration
            if rate < 1.0:
                reasons.append(f"sparse telemetry ({rate:.2f} Hz): poses interpolated across long gaps")
        return reasons

    # -- lookup --------------------------------------------------------

    def sample_at(self, t: float) -> TelemetrySample | None:
        """Linearly interpolate telemetry at time `t` (seconds since video start).

        Angles interpolate on the shortest arc, so a yaw crossing 360 deg does
        not swing the camera the long way round.
        """
        if not self.samples:
            return None
        if len(self.samples) == 1:
            return self.samples[0]

        times = [s.t for s in self.samples]
        if t <= times[0]:
            return self.samples[0]
        if t >= times[-1]:
            return self.samples[-1]

        i = bisect.bisect_left(times, t)
        a, b = self.samples[i - 1], self.samples[i]
        span = b.t - a.t
        w = 0.0 if span <= 0 else (t - a.t) / span

        def lerp(x, y):
            return None if (x is None or y is None) else x + (y - x) * w

        def lerp_ang(x, y):
            """Interpolate along the shortest arc, result normalized to [0, 360)."""
            if x is None or y is None:
                return None
            d = ((y - x) + 180.0) % 360.0 - 180.0
            return (x + d * w) % 360.0

        return TelemetrySample(
            t=t,
            lat=lerp(a.lat, b.lat),
            lon=lerp(a.lon, b.lon),
            alt_abs=lerp(a.alt_abs, b.alt_abs),
            alt_rel=lerp(a.alt_rel, b.alt_rel),
            gimbal_yaw=lerp_ang(a.gimbal_yaw, b.gimbal_yaw),
            gimbal_pitch=lerp_ang(a.gimbal_pitch, b.gimbal_pitch),
            gimbal_roll=lerp_ang(a.gimbal_roll, b.gimbal_roll),
            drone_yaw=lerp_ang(a.drone_yaw, b.drone_yaw),
            drone_pitch=lerp_ang(a.drone_pitch, b.drone_pitch),
            drone_roll=lerp_ang(a.drone_roll, b.drone_roll),
            focal_len_mm=lerp(a.focal_len_mm, b.focal_len_mm),
            gps_accuracy=lerp(a.gps_accuracy, b.gps_accuracy),
        )

    def sample_at_frame(self, frame_idx: int) -> TelemetrySample | None:
        if not self.fps:
            return None
        return self.sample_at(frame_idx / self.fps)

    # -- serialization (the fixture set depends on this) ---------------

    def to_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")

    @classmethod
    def from_json(cls, path: str | Path) -> FlightTrack:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        intr = d.pop("intrinsics", None)
        samples = [TelemetrySample(**s) for s in d.pop("samples", [])]
        track = cls(samples=samples, **d)
        if intr is not None:
            if intr.get("distortion") is not None:
                intr["distortion"] = tuple(intr["distortion"])
            track.intrinsics = CameraIntrinsics(**intr)
        return track
