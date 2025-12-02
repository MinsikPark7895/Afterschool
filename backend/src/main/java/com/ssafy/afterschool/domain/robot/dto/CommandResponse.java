package com.ssafy.afterschool.domain.robot.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.Builder;
import lombok.Getter;


/**
 * 명령 실행 응답 DTO
 * - 로봇 명령 전송 후 응답 정보
 */
@Schema(description = "로봇 명령 실행 응답 정보")
@Getter
@Builder
public class CommandResponse {

    /**
     * 명령 타입
     * - "move_to", "start_patrol", "stop_patrol", "emergency_stop"
     */
    @Schema(description = "명령 타입", allowableValues = {"move_to", "start_patrol", "stop_patrol"})
    private String commandType;

    /**
     * 응답 메시지
     */
    @Schema(description = "응답 메시지")
    private String message;

    public static CommandResponse sent(String commandType, String message) {
        return CommandResponse.builder()
                .commandType(commandType)
                .message(message)
                .build();
    }
}
