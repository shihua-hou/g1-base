import sys
from pathlib import Path


G1_BASE_ROOT = Path(__file__).resolve().parents[1]
if str(G1_BASE_ROOT) not in sys.path:
    sys.path.insert(0, str(G1_BASE_ROOT))

from teach.script_runner import load_script


def test_load_script_wraps_direct_jsonl_motion_file():
    motion_path = G1_BASE_ROOT / "config" / "movement" / "motions" / "haorizi06.jsonl"

    script_path, defaults, steps = load_script(motion_path)

    assert script_path == motion_path.resolve()
    assert defaults == {"profile": "upper_waist_lock"}
    assert steps == [{"type": "motion", "path": str(motion_path.resolve())}]
