"""The Fail2Drive route sets must stay in step with the vendored split. No simulator."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
import f2d_score as f2d  # noqa: E402

SPLIT_DIR = REPO / "third_party/fail2drive" / "fail2drive_split"
ROUTES = REPO / "benchmark" / "routes"


def read_set(name: str) -> list[str]:
    """Route names from a benchmark/routes/*.txt file: one per line, `#` starts a comment."""
    out = []
    for line in (ROUTES / name).read_text().splitlines():
        entry = line.split("#", 1)[0].strip()
        if entry:
            out.append(entry)
    return out


@pytest.mark.parametrize("name", ["f2d_pair.txt", "f2d_smoke.txt", "f2d_full.txt"])
def test_every_listed_route_exists(name):
    for route in read_set(name):
        assert (SPLIT_DIR / f"{route}.xml").is_file(), route


@pytest.mark.parametrize("name", ["f2d_pair.txt", "f2d_smoke.txt", "f2d_full.txt"])
def test_sets_are_complete_pairs(name):
    """Fail2Drive's headline number is a Base -> Generalization drop, so a set with an unpaired
    route could not produce one for that scenario."""
    rm = f2d.route_map(SPLIT_DIR)
    ids = {int(route.rsplit("_", 1)[1]) for route in read_set(name)}
    for rid in ids:
        twin = rid + f2d.PAIR_OFFSET if rm[rid][0] == "Base" else rid - f2d.PAIR_OFFSET
        assert twin in ids, f"{name}: route {rid} has no twin"


def test_full_set_is_exactly_the_vendored_split():
    """Generated from sorted(fail2drive_split/*.xml); regenerate it if the vendoring changes."""
    assert read_set("f2d_full.txt") == sorted(p.stem for p in SPLIT_DIR.glob("*.xml"))
    assert len(read_set("f2d_full.txt")) == 200


def test_pair_set_is_the_lowest_id_pair():
    assert read_set("f2d_pair.txt") == ["Base_BadParking_0000", "Generalization_BadParking_1000"]


def test_smoke_set_covers_every_scenario_class():
    rm = f2d.route_map(SPLIT_DIR)
    routes = read_set("f2d_smoke.txt")
    classes = {f2d.SCENARIO_TO_CLASS[rm[int(r.rsplit("_", 1)[1])][1]] for r in routes}
    assert classes == set(f2d.CLASS_PAPER_ORDER)
    assert len(routes) == 2 * len(f2d.CLASS_PAPER_ORDER)


def test_subsets_are_nested():
    full = set(read_set("f2d_full.txt"))
    assert set(read_set("f2d_pair.txt")) <= set(read_set("f2d_smoke.txt")) <= full
