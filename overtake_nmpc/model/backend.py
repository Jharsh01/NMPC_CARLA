"""Math backends so the same model code runs on NumPy floats and CasADi symbols."""

from types import SimpleNamespace

import casadi as ca
import numpy as np

NUMPY = SimpleNamespace(
    sin=np.sin,
    cos=np.cos,
    tan=np.tan,
    atan=np.arctan,
    sqrt=np.sqrt,
    fabs=np.fabs,
    sign=np.sign,
    fmax=np.fmax,
    if_else=np.where,
    stack=lambda *v: np.array(v, dtype=float),
)

CASADI = SimpleNamespace(
    sin=ca.sin,
    cos=ca.cos,
    tan=ca.tan,
    atan=ca.atan,
    sqrt=ca.sqrt,
    fabs=ca.fabs,
    sign=ca.sign,
    fmax=ca.fmax,
    if_else=ca.if_else,
    stack=ca.vertcat,
)
