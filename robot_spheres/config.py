"""Configuration model for fitting collision spheres to an MJCF robot."""

from dataclasses import dataclass, field, replace
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SphereFitConfig:
    """All robot-specific inputs required by the sphere generator."""

    input_xml: Path
    output_xml: Path
    source_group: int = 3
    output_group: int = 3
    sphere_density: float = 1.0
    exact_sphere_counts: dict[str, int] = field(default_factory=dict)
    body_names: tuple[str, ...] = ()
    fit_type: str = "morphit"
    iterations: int = 200
    coverage_weight: float | None = None
    protrusion_weight: float | None = None
    compute_metrics: bool = False
    keep_source_collisions: bool = False
    visualization_only: bool = False
    rgba: str = "0 1 1 0.35"

    def validate(self) -> "SphereFitConfig":
        if self.source_group < 0 or self.output_group < 0:
            raise ValueError("geometry groups must be nonnegative")
        if self.sphere_density <= 0.0:
            raise ValueError("sphere_density must be positive")
        if self.iterations < 1:
            raise ValueError("iterations must be positive")
        invalid_counts = {
            body: count
            for body, count in self.exact_sphere_counts.items()
            if count < 1
        }
        if invalid_counts:
            raise ValueError(
                f"sphere counts must be positive: {invalid_counts}"
            )
        return self

    def with_overrides(self, **values: Any) -> "SphereFitConfig":
        """Return a validated copy with non-None CLI overrides applied."""
        filtered = {
            key: value for key, value in values.items() if value is not None
        }
        return replace(self, **filtered).validate()

    @classmethod
    def from_json(cls, path: Path | str) -> "SphereFitConfig":
        """Load a profile, resolving its XML paths relative to the JSON file."""
        profile_path = Path(path).expanduser().resolve()
        values = json.loads(profile_path.read_text(encoding="utf-8"))
        base = profile_path.parent
        for key in ("input_xml", "output_xml"):
            value = Path(values[key]).expanduser()
            values[key] = value if value.is_absolute() else (base / value).resolve()
        values["body_names"] = tuple(values.get("body_names", ()))
        values["exact_sphere_counts"] = dict(
            values.get("exact_sphere_counts", {})
        )
        return cls(**values).validate()

    def to_json_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable representation useful for tooling."""
        return {
            "input_xml": str(self.input_xml),
            "output_xml": str(self.output_xml),
            "source_group": self.source_group,
            "output_group": self.output_group,
            "sphere_density": self.sphere_density,
            "exact_sphere_counts": self.exact_sphere_counts,
            "body_names": list(self.body_names),
            "fit_type": self.fit_type,
            "iterations": self.iterations,
            "coverage_weight": self.coverage_weight,
            "protrusion_weight": self.protrusion_weight,
            "compute_metrics": self.compute_metrics,
            "keep_source_collisions": self.keep_source_collisions,
            "visualization_only": self.visualization_only,
            "rgba": self.rgba,
        }
