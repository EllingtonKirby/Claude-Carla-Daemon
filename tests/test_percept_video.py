"""What a percept video is built from, without invoking ffmpeg.

The encoding is ffmpeg's problem; what this repo has to get right is which frame directories become
panels and in what order, because a percept writes a directory per DECISION and a drifting or
out-of-order selection would silently show four different moments side by side.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.percept_video import CAMS, build, ffmpeg_cmd, panel_dirs  # noqa: E402


def _frames(session: Path, steps, cams=CAMS):
    for step in steps:
        d = session / "frames" / f"frame_{step:05d}"
        d.mkdir(parents=True)
        for cam in cams:
            (d / f"{cam}.jpg").write_bytes(b"")
    return session


def test_panels_are_in_step_order_not_string_order(tmp_path):
    # 100 sorts before 20 as a bare string; the zero padding is what makes name order step order.
    s = _frames(tmp_path, [0, 20, 100, 1000])
    assert [d.name for d in panel_dirs(s)] == ["frame_00000", "frame_00020", "frame_00100", "frame_01000"]


def test_a_frame_missing_a_camera_is_skipped_not_padded(tmp_path):
    s = _frames(tmp_path, [0, 40])
    _frames(s, [20], cams=("CAM_FRONT",))          # a percept that named only the front camera
    assert [d.name for d in panel_dirs(s)] == ["frame_00000", "frame_00040"]


def test_a_session_with_no_usable_frames_fails_without_running_ffmpeg(tmp_path, capsys):
    (tmp_path / "frames").mkdir()
    assert build(tmp_path, tmp_path / "out.mp4", 2.0) is False
    assert "no frame directory" in capsys.readouterr().err


@pytest.mark.parametrize("cam", CAMS)
def test_every_camera_is_an_input_and_the_filter_consumes_them_all(tmp_path, cam):
    cmd = ffmpeg_cmd(tmp_path, tmp_path / "out.mp4", 2.0)
    assert any(f"frame_*/{cam}.jpg" in a for a in cmd)
    chain = cmd[cmd.index("-filter_complex") + 1]
    for i in range(len(CAMS)):
        assert f"[{i}:v]" in chain
    # Odd dimensions would be rejected by libx264 with yuv420p, which is what the panel is encoded as.
    assert "-pix_fmt" in cmd and cmd[cmd.index("-pix_fmt") + 1] == "yuv420p"


def test_the_panel_is_even_sized_so_libx264_accepts_it():
    from tools.percept_video import VIEW_H, VIEW_W
    assert (VIEW_W * 3) % 2 == 0 and (VIEW_H * 2) % 2 == 0
