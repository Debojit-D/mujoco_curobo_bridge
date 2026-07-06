# Robot collision-sphere toolkit

This package converts the collision geometry of any compiled MuJoCo MJCF
robot into body-local sphere approximations using cuRobo's sphere fitter. It
is intentionally independent of the Panda loader and can be copied as a
folder into another robot project.

The output is a complete, flattened MJCF. Every generated geom is named:

```text
spherefit_<body-name>_<index>
```

Its center is stored in that body's local frame, so the sphere follows the
link automatically when MuJoCo updates the robot configuration.

## What belongs to this package

```text
robot_spheres/
  __main__.py             python -m robot_spheres entry point
  cli.py                  CLI and profile overrides
  config.py               copyable SphereFitConfig data model
  generator.py            MJCF compilation, mesh assembly, fitting, export
  configs/
    full_robot_template.json
    another_robot_template.json
```

`demos/02_fit_mujoco_spheres.py` remains as a thin compatibility wrapper. New
robot integrations should use this package directly.

## Requirements

- A valid MuJoCo MJCF model
- Collision geoms assigned to a known MuJoCo geom group (group 3 here)
- `mujoco`, `numpy`, and `trimesh`
- cuRobo and its matching PyTorch/CUDA environment

The actual fit is GPU-backed and needs a visible CUDA-capable NVIDIA GPU.
`--help` and `--list-bodies` can still be used before cuRobo is installed.

From the repository root:

```bash
source .venv/bin/activate
python -m pip install -r mujoco_curobo_bridge/requirements.txt
```

Install cuRobo separately as described in the bridge README because its
PyTorch wheel must match the machine's CUDA version.

## Generate spheres for a complete robot

Copy the full-robot template and make its input/output paths point at your
robot-only MJCF. Run the package from its parent directory so `robot_spheres`
is importable:

```bash
cd mujoco_curobo_bridge
python -m robot_spheres \
  --config robot_spheres/configs/full_robot_template.json
```

An empty `body_names` list means: fit every body containing source-group
collision geometry. This includes complete arms, hands, and fingers when they
are present in the robot-only MJCF, without fitting tables or other environment
objects.

Before starting an expensive fit, inspect what will be included:

```bash
python -m robot_spheres \
  --config robot_spheres/configs/full_robot_template.json \
  --list-bodies
```

The profile writes a `.generated.xml` file so the checked-in fitted model is
not overwritten accidentally. Pass `--output-xml` when replacement is
intentional.

## Fit a different robot

1. Copy the `robot_spheres/` folder into the new project.
2. Copy `configs/another_robot_template.json` to a robot-specific filename.
3. Point `input_xml` at a robot-only MJCF scene.
4. Ensure the robot's collision geoms use `source_group` (normally group 3).
5. Run `--list-bodies` and check that every intended link appears.
6. Run the full fit and inspect coverage/protrusion metrics.
7. Compile and visualize the generated MJCF before using it in optimization.

Paths inside a JSON profile are resolved relative to the profile file, not
the current terminal directory. This is what makes a profile portable.

Example:

```bash
cd /path/to/new/project
python -m robot_spheres \
  --config robot_spheres/configs/my_robot.json
```

To fit only selected links:

```json
"body_names": ["upper_arm", "forearm", "wrist"]
```

To force a specific sphere count on difficult links:

```json
"exact_sphere_counts": {
  "forearm": 18,
  "wrist": 10
}
```

CLI overrides are useful for experiments without editing the profile:

```bash
python -m robot_spheres \
  --config robot_spheres/configs/my_robot.json \
  --sphere-density 1.5 \
  --iterations 300 \
  --num-spheres forearm=18 \
  --compute-metrics
```

## Choose the output mode deliberately

There are two independent decisions:

| Setting | Meaning |
|---|---|
| `keep_source_collisions=true` | Preserve the original MuJoCo collision geoms |
| `keep_source_collisions=false` | Disable source geoms that were sphere-fitted |
| `visualization_only=true` | Generated spheres have `contype=0`, `conaffinity=0` |
| `visualization_only=false` | Generated spheres participate in MuJoCo contacts |

For null-space proximity costs and viewer overlays, prefer:

```json
"keep_source_collisions": true,
"visualization_only": true
```

The original meshes remain responsible for physical MuJoCo contacts, while
the fitted spheres are read as a separate optimization model. This is how the
dual-Franka Equation 8 collision pipeline uses the generated MJCF.

To replace mesh collision with physical sphere collision, use:

```bash
python -m robot_spheres --config my_robot.json \
  --replace-source-collisions --contact-spheres
```

Do not enable both source and fitted contact geoms unintentionally; that
duplicates collision geometry and can produce excessive contact forces.

## How the fitter works

For each selected body, the generator:

1. Compiles the input MJCF with MuJoCo, resolving includes and defaults.
2. Selects direct child geoms in `source_group`.
3. Converts mesh and primitive geoms to one body-local triangle mesh.
4. Applies each geom's local position and quaternion.
5. Fits spheres with `curobo.sphere_fit.fit_spheres_to_mesh`.
6. Adds body-local sphere geoms to a flattened MJCF.
7. Compiles the output again as the final acceptance check.

The table/environment should normally be excluded by using a robot-only input
MJCF. Environment primitives such as a clean table box should be retained as
analytic obstacles rather than sphere-fitted.

## Consume spheres in another collision controller

The generated MJCF can be loaded as a sphere cache while the normal simulation
continues using its original model. For each `spherefit_*` geom, store:

- parent body name,
- `geom_pos` as the body-local center,
- `geom_size[0]` as radius.

Map a local center to world coordinates at runtime:

```python
rotation = data.xmat[body_id].reshape(3, 3)
world_center = data.xpos[body_id] + rotation @ local_center
```

Inter-arm avoidance compares every left-arm sphere with every right-arm
sphere. Same-arm self-collision needs a separate pair list that excludes the
same link and neighboring kinematic links; otherwise adjacent links create a
permanent false penalty.

The sphere penalty should remain a redundancy/null-space objective when the
primary controller is responsible for object pose or grasp tracking. Sphere
visualization alone does not enable avoidance.

## Connect generation to an Equation 8 controller

The intended controller path is:

1. `robot_spheres` generates body-local `spherefit_*` geoms.
2. The integrating project loads those geoms into a collision-sphere cache.
3. `_inter_arm_clearances()` transforms their centers into the world frame
   and evaluates all selected left-versus-right sphere pairs.
4. `inter_arm_collision_cost()` applies the smooth proximity penalty.
5. `value()` combines that penalty with the paper objective whose gradient is
   projected through Equation 8's null space.

This separation is intentional: the generated MJCF describes robot geometry;
the optimizer decides which pairs are meaningful and how strongly to penalize
them. For a different dual-arm robot, configure the left/right body
classification in that project's optimizer after generating the new spheres.
For same-arm self-collision, provide a curated non-adjacent link pair list
rather than comparing every sphere on that arm.

The `Bi-Manual_Redundancy_Work` repository is a concrete integration example:
its Equation 8 optimizer loads fitted sphere geoms, computes world-space
left-versus-right clearances, and applies the smooth collision cost only in the
projected redundancy objective.

## Python API

```python
from pathlib import Path
from robot_spheres import SphereFitConfig
from robot_spheres.generator import fit_mjcf

config = SphereFitConfig(
    input_xml=Path("robot_only_scene.xml"),
    output_xml=Path("robot_only_spherefit.xml"),
    body_names=(),                 # all bodies with source-group geoms
    sphere_density=1.0,
    compute_metrics=True,
    keep_source_collisions=True,
    visualization_only=True,
)
fit_mjcf(config)
```

## Validation checklist

- `--list-bodies` includes every intended robot link and no environment body.
- The fitter reports nonzero spheres for every selected body.
- Coverage is high enough for the intended safety margin.
- Protrusion is not so large that the robot becomes excessively conservative.
- The generated MJCF compiles successfully.
- Spheres move with their correct parent links in the viewer.
- Runtime minimum clearance is tested over the complete motion, not one pose.
- Grasp links are filtered downstream when they are expected to contact the
  manipulated object.

## Common failures

- **No bodies found:** collision geoms are not in `source_group`.
- **CUDA device error:** run the fit where the NVIDIA GPU and matching
  CUDA/PyTorch/cuRobo stack are visible.
- **Unknown body:** use `--list-bodies`; compiled names may differ from source
  include-file assumptions.
- **Generated model cannot find meshes:** keep the generated file beside the
  input model, or correct the flattened MJCF's `meshdir`/mesh paths.
- **Too many spheres:** lower `sphere_density` or set exact counts.
- **Poor thin-link coverage:** increase density/count and iterations.
- **Robot appears too large:** inspect protrusion metrics before changing the
  collision safety margin.
- **Gripper cannot grasp:** exclude hand/finger spheres from that specific
  object/environment penalty; do not delete them from the full fitted model.
