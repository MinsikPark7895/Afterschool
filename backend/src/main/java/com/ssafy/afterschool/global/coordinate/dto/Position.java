package com.ssafy.afterschool.global.coordinate.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.Builder;
import lombok.Getter;

/**
 * 3D 좌표를 담는 클래스
 * ROS 좌표계 또는 픽셀 좌표계의 위치 정보를 표현합니다.
 */
@Schema(description = "3D 좌표 위치 정보")
@Getter
@Builder
public class Position {
    @Schema(description = "X 좌표", example = "100.5")
    private final double x;

    @Schema(description = "Y 좌표", example = "200.3")
    private final double y;

    @Schema(description = "Z 좌표 (대부분 0)", example = "0.0")
    private final double z;

    public Position(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}