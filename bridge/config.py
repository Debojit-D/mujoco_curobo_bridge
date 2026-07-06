import os
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]

# Compatibility default for projects that include this repository as a
# top-level submodule beside their MUJOCO directory. Standalone users should
# set MUJOCO_SCENE explicitly to their own robot scene.
INTEGRATION_SCENE = (
    BRIDGE_ROOT.parent
    / "MUJOCO"
    / "robot_descriptions"
    / "franka_emika_panda"
    / "scene.xml"
)

MUJOCO_SCENE = Path(
    os.environ.get("MUJOCO_SCENE", INTEGRATION_SCENE)
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
