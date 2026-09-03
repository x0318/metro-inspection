"""Pure tunnel engineering-location calculations for localized 3D points."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class SemanticLocation:
    chainage_m: float
    chainage: str
    segment_name: str
    segment_id: int
    segment_offset_m: float
    clock_position_hours: float
    structure_area: str


class TunnelSemanticProjector:
    """Convert a point in the tunnel frame to report-friendly coordinates."""

    def __init__(
        self,
        *,
        chainage_start_m: float,
        chainage_axis: str,
        chainage_sign: float,
        segment_start_id: int,
        segment_length_m: float,
        segment_name: str,
        clock_center_y_m: float,
        clock_center_z_m: float,
    ) -> None:
        normalized_axis = str(chainage_axis).strip().lower()
        if normalized_axis not in {"x", "y", "z"}:
            raise ValueError("chainage_axis must be one of: x, y, z")
        if float(segment_length_m) <= 0.0:
            raise ValueError("segment_length_m must be positive")
        if not str(segment_name).strip():
            raise ValueError("segment_name must not be empty")

        self.chainage_start_m = float(chainage_start_m)
        self.chainage_axis = normalized_axis
        self.chainage_sign = float(chainage_sign)
        self.segment_start_id = int(segment_start_id)
        self.segment_length_m = float(segment_length_m)
        self.segment_name = str(segment_name).strip()
        self.clock_center_y_m = float(clock_center_y_m)
        self.clock_center_z_m = float(clock_center_z_m)

    def project(self, point: Point3D) -> SemanticLocation:
        axis_value = {
            "x": point.x,
            "y": point.y,
            "z": point.z,
        }[self.chainage_axis]
        relative_chainage_m = self.chainage_sign * axis_value
        chainage_m = self.chainage_start_m + relative_chainage_m

        segment_index = math.floor(relative_chainage_m / self.segment_length_m)
        segment_id = self.segment_start_id + segment_index
        segment_offset_m = (
            relative_chainage_m - segment_index * self.segment_length_m
        )

        dy = point.y - self.clock_center_y_m
        dz = point.z - self.clock_center_z_m
        clock_angle_deg = (
            math.degrees(math.atan2(-dy, dz)) + 360.0
        ) % 360.0
        clock_position_hours = clock_angle_deg / 30.0

        if clock_angle_deg < 45.0 or clock_angle_deg >= 315.0:
            structure_area = "拱顶/顶部"
        elif clock_angle_deg < 135.0:
            structure_area = "右侧边墙"
        elif clock_angle_deg < 225.0:
            structure_area = "底部/道床附近"
        else:
            structure_area = "左侧边墙"

        return SemanticLocation(
            chainage_m=chainage_m,
            chainage=self.format_chainage(chainage_m),
            segment_name=self.segment_name,
            segment_id=segment_id,
            segment_offset_m=segment_offset_m,
            clock_position_hours=clock_position_hours,
            structure_area=structure_area,
        )

    @staticmethod
    def format_chainage(chainage_m: float) -> str:
        sign = "-" if chainage_m < 0.0 else ""
        absolute_m = abs(float(chainage_m))
        kilometer = int(absolute_m // 1000.0)
        meter = absolute_m - kilometer * 1000.0
        return f"{sign}K{kilometer}+{meter:07.3f}"
