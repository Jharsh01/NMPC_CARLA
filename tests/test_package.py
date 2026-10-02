import importlib

import pytest

SUBPACKAGES = ["model", "controllers", "sim", "scenarios", "safety"]


@pytest.mark.parametrize("name", SUBPACKAGES)
def test_subpackage_imports(name):
    importlib.import_module(f"overtake_nmpc.{name}")


def test_dependencies_import():
    import casadi  # noqa: F401
    import matplotlib  # noqa: F401
    import numpy  # noqa: F401
    import scipy  # noqa: F401
