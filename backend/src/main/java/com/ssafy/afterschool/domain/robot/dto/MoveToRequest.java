package com.ssafy.afterschool.domain.robot.dto;

import com.ssafy.afterschool.global.coordinate.dto.Position;
import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.NoArgsConstructor;

/**
 * 로봇 이동 명령 요청 DTO
 * - 특정 위치로 로봇 이동 명령
 * - 프론트엔드에서 픽셀 좌표로 전송하면 백엔드에서 ROS 좌표로 변환
 */
@Schema(description = "로봇 이동 명령 요청 정보")
@Getter
@NoArgsConstructor
public class MoveToRequest {

    /**
     * 목표 위치 (픽셀 좌표)
     * - 프론트엔드에서 지도 상의 픽셀 좌표로 전송
     * - 백엔드에서 ROS 좌표로 변환하여 MQTT 전송
     */
    @Schema(description = "목표 위치 (픽셀 좌표)", requiredMode = Schema.RequiredMode.REQUIRED)
    @NotNull(message = "목표 위치는 필수입니다")
    private Position targetPosition;
}
