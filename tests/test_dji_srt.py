"""Conformance tests for the DJI SRT parser.

The contract these lock down applies to every S0 parser:
  - a recognised format yields usable telemetry
  - an unrecognised one degrades with warnings instead of raising
"""

from pathlib import Path

import pytest

from aerovis.ingest.dji_srt import find_srt_for_video, parse_dji_srt

FIXTURES = Path(__file__).parent / "fixtures" / "srt"


def test_modern_extracts_all_fields():
    t = parse_dji_srt(FIXTURES / "modern_mavic3.SRT")
    assert len(t.samples) == 2
    assert t.source_format == "dji_srt"
    assert t.has_geo

    s = t.samples[0]
    assert s.lat == pytest.approx(12.971599)
    assert s.lon == pytest.approx(77.594566)
    # Both altitudes share one bracket: "[rel_alt: 50.100 abs_alt: 920.300]"
    assert s.alt_rel == pytest.approx(50.1)
    assert s.alt_abs == pytest.approx(920.3)
    assert s.gimbal_yaw == pytest.approx(350.0)
    assert s.gimbal_pitch == pytest.approx(-89.9)
    # DJI reports tenths of a millimetre: 240 -> 24.0mm
    assert s.focal_len_mm == pytest.approx(24.0)
    assert t.warnings == []


def test_phantom4_spaced_keys_and_ambiguous_altitude():
    t = parse_dji_srt(FIXTURES / "phantom4.SRT")
    assert len(t.samples) == 2
    # "[latitude : 12.9716]" -- space before the colon
    assert t.samples[0].lat == pytest.approx(12.9716)
    assert t.samples[0].alt_rel == pytest.approx(100.5)
    assert any("ambiguous" in w for w in t.warnings)


def test_legacy_gps_disambiguates_lat_lon_order():
    """GPS(77.59, 12.97, 20) is (lon, lat, sats) -- latitude is bounded at +-90."""
    t = parse_dji_srt(FIXTURES / "legacy_gps.SRT")
    assert len(t.samples) == 2
    assert t.samples[0].lat == pytest.approx(12.9716)
    assert t.samples[0].lon == pytest.approx(77.5946)
    assert t.samples[0].alt_rel == pytest.approx(100.5)


def test_malformed_degrades_without_raising():
    t = parse_dji_srt(FIXTURES / "malformed.SRT")
    assert t.samples == []
    assert not t.has_geo
    assert t.warnings
    assert any("scale-free" in r for r in t.degraded_reasons())


def test_timeline_is_zero_based_and_sorted():
    t = parse_dji_srt(FIXTURES / "modern_mavic3.SRT")
    assert t.samples[0].t == 0.0
    assert [s.t for s in t.samples] == sorted(s.t for s in t.samples)


def test_fps_inferred_from_cue_rate():
    t = parse_dji_srt(FIXTURES / "modern_mavic3.SRT")
    assert t.fps == pytest.approx(30.3, abs=0.5)


def test_interpolation_between_samples():
    t = parse_dji_srt(FIXTURES / "modern_mavic3.SRT")
    mid = t.sample_at(t.samples[1].t / 2)
    assert t.samples[0].lat < mid.lat < t.samples[1].lat
    # Yaw 350 -> 10 must cross through 0, not swing back through 180.
    assert mid.gimbal_yaw == pytest.approx(0.0, abs=0.5)


def test_sample_at_clamps_outside_range():
    t = parse_dji_srt(FIXTURES / "modern_mavic3.SRT")
    assert t.sample_at(-5.0).lat == t.samples[0].lat
    assert t.sample_at(1e6).lat == t.samples[-1].lat


def test_roundtrip_through_json(tmp_path):
    t = parse_dji_srt(FIXTURES / "modern_mavic3.SRT")
    p = tmp_path / "track.json"
    t.to_json(p)

    from aerovis.core.schema import FlightTrack
    back = FlightTrack.from_json(p)
    assert len(back.samples) == len(t.samples)
    assert back.samples[0].lat == t.samples[0].lat
    assert back.source_format == t.source_format


def test_find_srt_for_video_handles_casing(tmp_path):
    (tmp_path / "DJI_0001.MP4").write_bytes(b"")
    (tmp_path / "DJI_0001.SRT").write_text("")
    assert find_srt_for_video(tmp_path / "DJI_0001.MP4").name == "DJI_0001.SRT"
    assert find_srt_for_video(tmp_path / "nothing.MP4") is None
