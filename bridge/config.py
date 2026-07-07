import os
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
PANDA_SCENE_RELATIVE = Path("robot_descriptions/franka_emika_panda/scene.xml")


def _default_scene_candidates():
    """Yield common integration layouts from nearest to farthest parent."""
    for root in (BRIDGE_ROOT, *BRIDGE_ROOT.parents):
        yield root / PANDA_SCENE_RELATIVE
        yield root / "MUJOCO" / PANDA_SCENE_RELATIVE


def _default_mujoco_scene():
    for candidate in _default_scene_candidates():
        if candidate.is_file():
            return candidate.resolve()
    return (BRIDGE_ROOT.parent / PANDA_SCENE_RELATIVE).resolve()

MUJOCO_SCENE = Path(
    os.environ.get("MUJOCO_SCENE", _default_mujoco_scene())
).expanduser().resolve()


def require_mujoco_scene(path=MUJOCO_SCENE):
    """Return a configured scene or explain how a standalone user supplies it."""
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(
            f"MuJoCo scene not found: {path}. Set MUJOCO_SCENE to a valid "
            "MJCF/XML robot scene."
        )
    return path
