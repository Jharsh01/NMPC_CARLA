import pytest

from overtake_core import (
    BicyclePlant,
    ChronoPlant,
    Controller,
    NMPC,
    PlantModel,
    PurePursuit,
    TrackNMPC,
    TrackPurePursuit,
    VehicleParams,
)


def test_python_side_imports():
    import imageio_ffmpeg  # noqa: F401
    import matplotlib  # noqa: F401
    import numpy  # noqa: F401

    import sim.scenarios.circuit.animate  # noqa: F401
    import sim.scenarios.overtake.animate  # noqa: F401
    import sim.view  # noqa: F401


def test_controllers_and_plants_follow_their_interfaces():
    for controller in (NMPC, TrackNMPC, PurePursuit, TrackPurePursuit):
        assert issubclass(controller, Controller)
    assert isinstance(BicyclePlant(VehicleParams()), PlantModel)
    assert issubclass(ChronoPlant, PlantModel)
