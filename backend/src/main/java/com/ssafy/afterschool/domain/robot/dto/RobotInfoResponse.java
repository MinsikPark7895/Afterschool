package com.ssafy.afterschool.domain.robot.dto;

import com.ssafy.afterschool.domain.robot.entity.Robot;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

/**
 * 로봇 정보 응답 DTO
 * - API에서 로봇 정보를 반환할 때 사용
 */
@Getter
@Builder
public class RobotInfoResponse {

    private Long id;
    private String robotId;
    private String robotName;
    private boolean isActive;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;

    public static RobotInfoResponse from(Robot robot) {
        return RobotInfoResponse.builder()
                .id(robot.getId())
                .robotId(robot.getRobotId())
                .robotName(robot.getRobotName())
                .isActive(robot.isActive())
                .createdAt(robot.getCreatedAt())
                .updatedAt(robot.getUpdatedAt())
                .build();
    }
}
