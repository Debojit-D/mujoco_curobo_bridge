"""Fit collision spheres to the selected MuJoCo scene.

This compatibility entry point delegates to the reusable ``robot_spheres``
package. Existing command-line options remain available, while new robots can
use a JSON profile through ``--config``. Run with ``--help`` for details.
"""

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.config import MUJOCO_SCENE
from robot_spheres.cli import main


if __name__ == "__main__":
    main(default_input=MUJOCO_SCENE)
