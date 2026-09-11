import pytest

from metro_localization.semantic_geometry import Point3D, TunnelSemanticProjector


def make_projector():
    return TunnelSemanticProjector(
        chainage_start_m=12000.0,
        chainage_axis="x",
        chainage_sign=1.0,
        segment_start_id=1000,
        segment_length_m=1.2,
        segment_name="仿真环号",
        clock_center_y_m=0.0,
        clock_center_z_m=1.75,
    )


def test_projects_chainage_ring_and_crown_location():
    result = make_projector().project(Point3D(x=2.5, y=0.0, z=2.75))

    assert result.chainage_m == pytest.approx(12002.5)
    assert result.chainage == "K12+002.500"
    assert result.segment_id == 1002
    assert result.segment_offset_m == pytest.approx(0.1)
    assert result.clock_position_hours == pytest.approx(0.0)
    assert result.structure_area == "拱顶/顶部"


def test_projects_right_wall_to_three_oclock():
    result = make_projector().project(Point3D(x=0.0, y=-1.0, z=1.75))

    assert result.clock_position_hours == pytest.approx(3.0)
    assert result.structure_area == "右侧边墙"
