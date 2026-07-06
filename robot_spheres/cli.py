"""Command-line interface for the reusable robot sphere generator."""

import argparse
from pathlib import Path
from typing import Sequence

from .config import SphereFitConfig
from .generator import fit_mjcf, inspect_fittable_bodies


def _parse_exact_counts(values: Sequence[str] | None) -> dict[str, int] | None:
    if values is None:
        return None
    result = {}
    for value in values:
        try:
            body, count = value.rsplit("=", 1)
            result[body] = int(count)
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                f"expected BODY=COUNT, received {value!r}"
            ) from error
        if result[body] < 1:
            raise argparse.ArgumentTypeError("sphere count must be positive")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fit cuRobo spheres to MuJoCo collision geometry and write a "
            "complete flattened MJCF model."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        help="JSON robot profile; relative XML paths use the profile directory",
    )
    parser.add_argument("--input-xml", type=Path)
    parser.add_argument("--output-xml", type=Path)
    parser.add_argument("--source-group", type=int)
    parser.add_argument("--output-group", type=int)
    parser.add_argument("--sphere-density", type=float)
    parser.add_argument(
        "--num-spheres",
        action="append",
        metavar="BODY=COUNT",
        help="exact count for one body; repeat for multiple bodies",
    )
    parser.add_argument(
        "--body",
        action="append",
        help="fit only this body; repeatable; omitted means every fittable body",
    )
    parser.add_argument("--fit-type")
    parser.add_argument("--iterations", type=int)
    parser.add_argument("--coverage-weight", type=float)
    parser.add_argument("--protrusion-weight", type=float)
    parser.add_argument(
        "--compute-metrics",
        action="store_true",
        default=None,
    )
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument(
        "--keep-source-collisions",
        action="store_true",
        default=None,
    )
    source_group.add_argument(
        "--replace-source-collisions",
        action="store_true",
        default=None,
        help="disable fitted source geoms in the generated MJCF",
    )
    contact_mode = parser.add_mutually_exclusive_group()
    contact_mode.add_argument(
        "--visualization-only",
        action="store_true",
        default=None,
        help="write non-contact spheres for overlays/optimization models",
    )
    contact_mode.add_argument(
        "--contact-spheres",
        action="store_true",
        default=None,
        help="make fitted spheres active MuJoCo collision geoms",
    )
    parser.add_argument("--rgba")
    parser.add_argument(
        "--list-bodies",
        action="store_true",
        help="compile the input and list bodies with source-group geometry",
    )
    return parser


def config_from_args(
    args: argparse.Namespace,
    *,
    default_input: Path | None = None,
) -> SphereFitConfig:
    if args.config is not None:
        config = SphereFitConfig.from_json(args.config)
    else:
        input_xml = args.input_xml or default_input
        if input_xml is None:
            raise ValueError("provide --config or --input-xml")
        input_xml = Path(input_xml).expanduser().resolve()
        output_xml = args.output_xml or input_xml.with_name(
            f"{input_xml.stem}_spherefit.xml"
        )
        config = SphereFitConfig(input_xml=input_xml, output_xml=output_xml)

    keep_source_collisions = None
    if args.keep_source_collisions:
        keep_source_collisions = True
    elif args.replace_source_collisions:
        keep_source_collisions = False
    visualization_only = None
    if args.visualization_only:
        visualization_only = True
    elif args.contact_spheres:
        visualization_only = False

    return config.with_overrides(
        input_xml=(args.input_xml.expanduser().resolve() if args.input_xml else None),
        output_xml=(
            args.output_xml.expanduser().resolve() if args.output_xml else None
        ),
        source_group=args.source_group,
        output_group=args.output_group,
        sphere_density=args.sphere_density,
        exact_sphere_counts=_parse_exact_counts(args.num_spheres),
        body_names=tuple(args.body) if args.body is not None else None,
        fit_type=args.fit_type,
        iterations=args.iterations,
        coverage_weight=args.coverage_weight,
        protrusion_weight=args.protrusion_weight,
        compute_metrics=args.compute_metrics,
        keep_source_collisions=keep_source_collisions,
        visualization_only=visualization_only,
        rgba=args.rgba,
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    default_input: Path | None = None,
) -> Path | None:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = config_from_args(args, default_input=default_input)
    except (KeyError, TypeError, ValueError) as error:
        parser.error(str(error))

    if args.list_bodies:
        bodies = inspect_fittable_bodies(
            config.input_xml,
            config.source_group,
        )
        print(f"Fittable bodies in {config.input_xml}:")
        for body_name, geom_count in bodies.items():
            print(f"  {body_name}: {geom_count} source geoms")
        return None
    return fit_mjcf(config)
