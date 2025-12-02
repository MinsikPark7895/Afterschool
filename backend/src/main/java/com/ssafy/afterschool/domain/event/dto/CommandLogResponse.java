package com.ssafy.afterschool.domain.event.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

/**
 * 명령 로그 응답 DTO
 */
@Getter
@Builder
public class CommandLogResponse {

    private Long id;

    @JsonProperty("targetRobotId")
    private String targetRobotId;

    @JsonProperty("userId")
    private Long userId;

    @JsonProperty("commandType")
    private String commandType;

    @JsonProperty("commandPayload")
    private String commandPayload; // JSON 문자열

    @JsonProperty("commandStatus")
    private String commandStatus;

    @JsonProperty("responseData")
    private String responseData; // JSON 문자열

    @JsonProperty("sentAt")
    private LocalDateTime sentAt;

    @JsonProperty("completedAt")
    private LocalDateTime completedAt;

    @JsonProperty("executionTime")
    private Long executionTime; // 실행 시간 (초)
}
