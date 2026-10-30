"""Pydantic models for the annotations sidecar."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import BaseModel, Field, field_validator


class DoorAnnotation(BaseModel):
    """Field-compatible with spot_arm_door_fiducial.config.DoorConfig.

    Only fiducials in the localization range (1-299) may be used as doors;
    dock/mission-interrupt tags are reserved by Boston Dynamics.
    """
    name: str
    tag_id: int
    tag_to_handle: tuple[float, float, float] = (0.0, 0.0, 0.12)
    handle_type: Literal['lever', 'push_bar'] = 'lever'
    swing: Literal['push', 'pull'] = 'pull'
    hinge: Literal['left', 'right'] = 'right'
    nav_waypoint: str | None = None
    approach_standoff_m: float = 1.0

    @field_validator('tag_id')
    @classmethod
    def _localization_range(cls, v: int) -> int:
        if not 1 <= v <= 299:
            raise ValueError(f'door tag_id must be in 1-299 (localization range); got {v}')
        return v


class WaypointPOI(BaseModel):
    name: str = ''
    tags: list[str] = Field(default_factory=list)
    notes: str = ''


class Polygon(BaseModel):
    name: str
    polygon_seed: list[tuple[float, float]]   # [[x, y], ...] in seed frame
    waypoints: list[str] = Field(default_factory=list)
    color: str | None = None   # "#rrggbb"; None → default green at render time


class NoGoZone(BaseModel):
    name: str
    polygon_seed: list[tuple[float, float]]


class PatrolStop(BaseModel):
    waypoint_id: str
    # Map-frame heading to face on arrival (0 = +x, CCW positive), or None to
    # keep the waypoint's recorded orientation.
    heading_deg: float | None = None
    dwell_s: float = Field(default=5.0, ge=0, le=600)

    @field_validator('heading_deg')
    @classmethod
    def _normalize_heading(cls, v: float | None) -> float | None:
        if v is None:
            return None
        if not math.isfinite(v):
            raise ValueError('heading_deg must be a finite number')
        return v % 360.0


class Patrol(BaseModel):
    """An ordered list of stops the robot visits in sequence."""
    name: str
    stops: list[PatrolStop] = Field(default_factory=list)
    loop: bool = False   # start over from the first stop until stopped


class Annotations(BaseModel):
    doors: list[DoorAnnotation] = Field(default_factory=list)
    waypoint_pois: dict[str, WaypointPOI] = Field(default_factory=dict)
    rooms: list[Polygon] = Field(default_factory=list)
    no_go_zones: list[NoGoZone] = Field(default_factory=list)
    patrols: list[Patrol] = Field(default_factory=list)
