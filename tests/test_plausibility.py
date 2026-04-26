"""Per-body physical plausibility checks across every URDF and MJCF.

For each body we verify mass is positive and bounded, principal inertias are
positive and bounded, and they satisfy the triangle inequality. Zero-tensor
frame links (e.g. fingertip markers) are intentionally exempted from the
inertia checks.
"""
import pytest

from .conftest import (
    assert_body_plausible,
    body_params,
    discover_mjcfs,
    discover_urdfs,
    parse_mjcf,
    parse_urdf,
)

URDF_PARAMS, URDF_IDS = body_params(parse_urdf, discover_urdfs())
MJCF_PARAMS, MJCF_IDS = body_params(parse_mjcf, discover_mjcfs())


@pytest.mark.skipif(not URDF_PARAMS, reason="no URDF bodies")
@pytest.mark.parametrize(("model", "name"), URDF_PARAMS, ids=URDF_IDS)
def test_urdf_body_plausible(model, name):
    assert_body_plausible(name, model.bodies[name], model.label)


@pytest.mark.skipif(not MJCF_PARAMS, reason="no MJCF bodies")
@pytest.mark.parametrize(("model", "name"), MJCF_PARAMS, ids=MJCF_IDS)
def test_mjcf_body_plausible(model, name):
    assert_body_plausible(name, model.bodies[name], model.label)
