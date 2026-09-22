# -*- coding: utf-8 -*-
"""Hold gantry R-axis at the angle captured on first motion."""
from __future__ import annotations

from typing import Any, Mapping, MutableMapping


class RAxisHold:
    """Freeze yaw/R after the first observed pose so later moves only change XYZ."""

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = bool(enabled)
        self.locked_yaw_deg: float | None = None
        self.locked_r_deg: float | None = None

    def reset(self) -> None:
        self.locked_yaw_deg = None
        self.locked_r_deg = None

    def capture_from_pose(
        self,
        pose: Mapping[str, Any],
        gantry_xyzr: Mapping[str, Any] | None = None,
    ) -> None:
        if not self.enabled or self.locked_yaw_deg is not None:
            return
        self.locked_yaw_deg = float(pose.get("yaw_deg", pose.get("yaw", pose.get("R", 0.0))) or 0.0)
        if isinstance(gantry_xyzr, Mapping) and gantry_xyzr.get("R") is not None:
            self.locked_r_deg = float(gantry_xyzr["R"])
        else:
            self.locked_r_deg = self.locked_yaw_deg

    def apply_to_pose(self, pose: Mapping[str, Any]) -> dict[str, Any]:
        out = dict(pose or {})
        if self.enabled and self.locked_yaw_deg is not None:
            out["yaw_deg"] = float(self.locked_yaw_deg)
        return out

    def apply_to_gantry(self, gantry_xyzr: MutableMapping[str, Any] | None) -> dict[str, float] | None:
        if not isinstance(gantry_xyzr, Mapping):
            return None
        out = {key: float(gantry_xyzr[key]) for key in ("X", "Y", "Z", "R") if key in gantry_xyzr}
        if self.enabled and self.locked_r_deg is not None:
            out["R"] = float(self.locked_r_deg)
        return out
