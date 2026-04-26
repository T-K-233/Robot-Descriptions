"""Left/right symmetry checks across every URDF and MJCF in the repo."""
import pytest

from .conftest import (
    assert_inertia_close,
    assert_mass_close,
    assert_ranges_match,
    body_pair_params,
    discover_mjcfs,
    discover_urdfs,
    joint_pair_params,
    parse_mjcf,
    parse_urdf,
)

URDF_BODY_PARAMS, URDF_BODY_IDS = body_pair_params(parse_urdf, discover_urdfs())
URDF_JOINT_PARAMS, URDF_JOINT_IDS = joint_pair_params(parse_urdf, discover_urdfs())
MJCF_BODY_PARAMS, MJCF_BODY_IDS = body_pair_params(parse_mjcf, discover_mjcfs())
MJCF_JOINT_PARAMS, MJCF_JOINT_IDS = joint_pair_params(parse_mjcf, discover_mjcfs())


@pytest.mark.skipif(not URDF_BODY_PARAMS, reason="no URDF left/right body pairs")
@pytest.mark.parametrize(("model", "left", "right"), URDF_BODY_PARAMS, ids=URDF_BODY_IDS)
def test_urdf_body_mass_symmetric(model, left, right):
    assert_mass_close(model.bodies[left], model.bodies[right], model.label)


@pytest.mark.skipif(not URDF_BODY_PARAMS, reason="no URDF left/right body pairs")
@pytest.mark.parametrize(("model", "left", "right"), URDF_BODY_PARAMS, ids=URDF_BODY_IDS)
def test_urdf_body_inertia_symmetric(model, left, right):
    assert_inertia_close(model.bodies[left], model.bodies[right], model.label)


@pytest.mark.skipif(not URDF_JOINT_PARAMS, reason="no URDF left/right joint pairs")
@pytest.mark.parametrize(("model", "left", "right"), URDF_JOINT_PARAMS, ids=URDF_JOINT_IDS)
def test_urdf_joint_range_symmetric(model, left, right):
    assert_ranges_match(
        model.joint_ranges[left], model.joint_ranges[right], model.label, (left, right)
    )


@pytest.mark.skipif(not MJCF_BODY_PARAMS, reason="no MJCF left/right body pairs")
@pytest.mark.parametrize(("model", "left", "right"), MJCF_BODY_PARAMS, ids=MJCF_BODY_IDS)
def test_mjcf_body_mass_symmetric(model, left, right):
    assert_mass_close(model.bodies[left], model.bodies[right], model.label)


@pytest.mark.skipif(not MJCF_BODY_PARAMS, reason="no MJCF left/right body pairs")
@pytest.mark.parametrize(("model", "left", "right"), MJCF_BODY_PARAMS, ids=MJCF_BODY_IDS)
def test_mjcf_body_inertia_symmetric(model, left, right):
    assert_inertia_close(model.bodies[left], model.bodies[right], model.label)


@pytest.mark.skipif(not MJCF_JOINT_PARAMS, reason="no MJCF left/right joint pairs")
@pytest.mark.parametrize(("model", "left", "right"), MJCF_JOINT_PARAMS, ids=MJCF_JOINT_IDS)
def test_mjcf_joint_range_symmetric(model, left, right):
    assert_ranges_match(
        model.joint_ranges[left], model.joint_ranges[right], model.label, (left, right)
    )
