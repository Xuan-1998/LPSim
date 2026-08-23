"""Stable mode identifiers shared by multimode data files."""

from __future__ import annotations

from enum import Enum


class Mode(str, Enum):
    """Modes that may appear in an OD itinerary."""

    CAR = "car"
    AV = "av"
    BUS = "bus"
    METRO = "metro"
    BIKE = "bike"
    WALK = "walk"
    UAM = "uam"
    RIDESHARE = "rideshare"


ROAD_MODES = frozenset({Mode.CAR, Mode.AV, Mode.BUS, Mode.RIDESHARE})
TRANSIT_MODES = frozenset({Mode.BUS, Mode.METRO})
ACTIVE_MODES = frozenset({Mode.WALK, Mode.BIKE})
AIR_MODES = frozenset({Mode.UAM})
