import json
from pathlib import Path

import pytest

import overtake_core
from overtake_core import NU, NX, PARAMETERS_PATH, VehicleParams, replace

STATE = ("X", "Y", "psi", "Ux", "Uy", "r", "delta")
INPUT = ("delta_rate", "Fx")


def test_defaults_come_from_the_file():
    data = json.loads(Path(PARAMETERS_PATH).read_text())
    p = VehicleParams()
    for name, entry in data["vehicle"]["parameters"].items():
        assert getattr(p, name) == entry["value"]
        assert entry["source"] in data["vehicle"]["sources"]
        assert entry["unit"] and entry["description"]
    assert p.L == pytest.approx(p.a + p.b)


def test_state_and_input_definition():
    data = json.loads(Path(PARAMETERS_PATH).read_text())
    assert tuple(entry["name"] for entry in data["state"]) == STATE and NX == 7
    assert tuple(entry["name"] for entry in data["input"]) == INPUT and NU == 2
    assert STATE[overtake_core.IUX] == "Ux" and INPUT[overtake_core.IFX] == "Fx"


def test_replace_and_other_files(tmp_path):
    p = VehicleParams()
    assert replace(p, mu=0.5).mu == 0.5 and p.mu == 0.9

    data = json.loads(Path(PARAMETERS_PATH).read_text())
    data["vehicle"]["parameters"]["m"]["value"] = 1500.0
    other = tmp_path / "other.json"
    other.write_text(json.dumps(data))
    assert VehicleParams.from_file(str(other)).m == 1500.0
    assert VehicleParams.from_file(str(other)).a == p.a

    data["state"][0], data["state"][1] = data["state"][1], data["state"][0]
    other.write_text(json.dumps(data))
    with pytest.raises(RuntimeError):  # the code is written for one state order
        VehicleParams.from_file(str(other))
