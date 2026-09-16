"""DJI SRT sidecar parser -- the reference implementation for S0.

DJI writes a `.SRT` next to the video containing per-frame telemetry. The format
has drifted across product generations, so this parser is deliberately tolerant:
it extracts whatever `key: value` pairs it finds rather than matching one layout.

Three families are handled:

  modern (Mavic 3, Mini 3/4, Air 2S)
    [latitude: 12.97] [longitude: 77.59] [rel_alt: 50.1 abs_alt: 920.3]

  mid-era (Phantom 4)
    [latitude : 12.97] [longitude : 77.59] [altitude: 100.5]

  legacy (no brackets)
    GPS(77.5946,12.9716,20) BAROMETER:100.5

Write new parsers (MAVLink, CSV) against this shape: parse -> normalize ->
return a FlightTrack, recording anything suspicious in `warnings` rather than
raising. A parser that throws on unusual input fails the whole run; one that
degrades keeps the pipeline alive.
"""

from __future__ import annotations

import re
from pathlib import Path

from aerovis.core.schema import CameraIntrinsics, FlightTrack, TelemetrySample

# "00:00:01,234 --> 00:00:01,267"
_TIMECODE = re.compile(
    r"(\d{2}):(\d{2}):(\d{2})[,.](\d{1,3})\s*-->\s*(\d{2}):(\d{2}):(\d{2})[,.](\d{1,3})"
)
_HTML_TAG = re.compile(r"<[^>]+>")
_BRACKET = re.compile(r"\[([^\]]*)\]")
# Multiple pairs can share one bracket: "[rel_alt: 50.1 abs_alt: 920.3]"
_KV = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([^\s\]]+)")
# Legacy: GPS(lon,lat,sats)
_LEGACY_GPS = re.compile(r"GPS\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)")
_LEGACY_BARO = re.compile(r"BAROMETER\s*:\s*([-\d.]+)", re.IGNORECASE)

# Source key -> FlightTrack field. Lowercased before lookup.
_ALIASES = {
    "latitude": "lat", "lat": "lat",
    "longitude": "lon", "lon": "lon", "long": "lon",
    "rel_alt": "alt_rel", "relative_altitude": "alt_rel",
    "abs_alt": "alt_abs", "absolute_altitude": "alt_abs",
    "gb_yaw": "gimbal_yaw", "gimbal_yaw": "gimbal_yaw",
    "gb_pitch": "gimbal_pitch", "gimbal_pitch": "gimbal_pitch",
    "gb_roll": "gimbal_roll", "gimbal_roll": "gimbal_roll",
    "drone_yaw": "drone_yaw", "dr_yaw": "drone_yaw",
    "drone_pitch": "drone_pitch", "dr_pitch": "drone_pitch",
    "drone_roll": "drone_roll", "dr_roll": "drone_roll",
}


def _as_float(text: str) -> float | None:
    try:
        return float(text)
    except ValueError:
        return None


def _timecode_seconds(line: str) -> float | None:
    """Start time of an SRT cue, in seconds."""
    m = _TIMECODE.search(line)
    if not m:
        return None
    h, mi, s, ms = (int(m.group(i)) for i in range(1, 5))
    return h * 3600 + mi * 60 + s + ms / 1000.0


def _split_blocks(text: str) -> list[list[str]]:
    """Split an SRT into cue blocks on blank lines."""
    blocks, current = [], []
    for raw in text.splitlines():
        if raw.strip():
            current.append(raw)
        elif current:
            blocks.append(current)
            current = []
    if current:
        blocks.append(current)
    return blocks


def _parse_block(lines: list[str], warnings: list[str]) -> TelemetrySample | None:
    t = next((tc for line in lines if (tc := _timecode_seconds(line)) is not None), None)
    if t is None:
        return None

    payload = _HTML_TAG.sub(" ", " ".join(lines))
    sample = TelemetrySample(t=t)
    found = {}

    for bracket in _BRACKET.findall(payload):
        for key, value in _KV.findall(bracket):
            found[key.lower()] = value

    for key, value in found.items():
        field = _ALIASES.get(key)
        if field and (v := _as_float(value)) is not None:
            setattr(sample, field, v)

    # DJI reports focal length in tenths of a millimetre (240 -> 24.0mm).
    if (fl := _as_float(found.get("focal_len", ""))) is not None:
        sample.focal_len_mm = fl / 10.0 if fl >= 100 else fl

    # Mid-era firmware writes a bare `altitude` whose datum is ambiguous across
    # models. Record it as relative; a constant Z offset is absorbed by the
    # S3 similarity transform either way.
    if sample.alt_rel is None and sample.alt_abs is None:
        if (alt := _as_float(found.get("altitude", ""))) is not None:
            sample.alt_rel = alt
            if "ambiguous `altitude` field: assumed relative to takeoff" not in warnings:
                warnings.append("ambiguous `altitude` field: assumed relative to takeoff")

    # Legacy bracket-free form.
    if not sample.has_position:
        if m := _LEGACY_GPS.search(payload):
            a, b = float(m.group(1)), float(m.group(2))
            # Documented as GPS(lon,lat,sats), but ordering varies by firmware.
            # Latitude is bounded at +-90, so use that to disambiguate.
            if abs(a) <= 90 < abs(b):
                sample.lat, sample.lon = a, b
            else:
                sample.lon, sample.lat = a, b
        if m := _LEGACY_BARO.search(payload):
            sample.alt_rel = float(m.group(1))

    return sample


def parse_dji_srt(srt_path: str | Path, video_path: str | Path | None = None) -> FlightTrack:
    """Parse a DJI `.SRT` sidecar into a `FlightTrack`.

    Never raises on malformed content -- unparseable cues are counted in
    `track.warnings` so the run continues in degraded mode.
    """
    srt_path = Path(srt_path)
    text = srt_path.read_text(encoding="utf-8", errors="replace")

    warnings: list[str] = []
    blocks = _split_blocks(text)
    samples = [s for b in blocks if (s := _parse_block(b, warnings)) is not None]

    if skipped := len(blocks) - len(samples):
        warnings.append(f"{skipped} of {len(blocks)} SRT cues had no readable timecode")

    samples.sort(key=lambda s: s.t)

    # Shift so the first cue is t=0, matching frame indexing downstream.
    if samples and samples[0].t != 0:
        offset = samples[0].t
        for s in samples:
            s.t -= offset

    track = FlightTrack(
        video_path=str(video_path) if video_path else str(srt_path.with_suffix(".MP4")),
        samples=samples,
        source_format="dji_srt",
        warnings=warnings,
    )

    if len(samples) >= 2 and track.duration > 0:
        track.fps = round((len(samples) - 1) / track.duration, 3)

    if not samples:
        track.warnings.append("no telemetry recovered from SRT")
    elif not track.has_geo:
        track.warnings.append("SRT parsed but contains no GPS positions")

    return track


def find_srt_for_video(video_path: str | Path) -> Path | None:
    """Locate the SRT beside a video. DJI casing is inconsistent across models."""
    video_path = Path(video_path)
    for suffix in (".SRT", ".srt", ".Srt"):
        if (candidate := video_path.with_suffix(suffix)).exists():
            return candidate
    return None
