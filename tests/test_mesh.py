"""Verify every mesh referenced in a URDF or MJCF resolves to a real file."""
import pytest

from .conftest import (
    discover_mjcfs,
    discover_urdfs,
    mesh_params,
    mjcf_mesh_refs,
    urdf_mesh_refs,
)

URDF_PARAMS, URDF_IDS = mesh_params(urdf_mesh_refs, discover_urdfs())
MJCF_PARAMS, MJCF_IDS = mesh_params(mjcf_mesh_refs, discover_mjcfs())


@pytest.mark.skipif(not URDF_PARAMS, reason="no URDF mesh references")
@pytest.mark.parametrize("ref", URDF_PARAMS, ids=URDF_IDS)
def test_urdf_mesh_exists(ref):
    assert ref.resolved.is_file(), (
        f"[{ref.source.name}] missing mesh: {ref.name} -> {ref.resolved}"
    )


@pytest.mark.skipif(not MJCF_PARAMS, reason="no MJCF mesh references")
@pytest.mark.parametrize("ref", MJCF_PARAMS, ids=MJCF_IDS)
def test_mjcf_mesh_exists(ref):
    assert ref.resolved.is_file(), (
        f"[{ref.source.name}] missing mesh: {ref.name} -> {ref.resolved}"
    )
