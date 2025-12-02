package com.ssafy.afterschool.domain.event.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.ssafy.afterschool.domain.event.entity.Event;
import com.ssafy.afterschool.domain.event.entity.EventType;
import com.ssafy.afterschool.domain.event.entity.Severity;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;
import java.util.List;

/**
 * 이벤트 응답 DTO
 */
@Schema(description = "이벤트 상세 정보 응답")
@Getter
@Builder
public class EventResponse {

    @Schema(description = "이벤트 ID", example = "1")
    private Long id;

    @Schema(description = "이벤트 타입", example = "DETECTION")
    @JsonProperty("eventType")
    private EventType eventType;

    @Schema(description = "로봇 ID", example = "robot_01")
    @JsonProperty("robotId")
    private String robotId;

    @Schema(description = "심각도", example = "CRITICAL")
    @JsonProperty("severity")
    private Severity severity;

    @JsonProperty("locationData")
    private String locationData; // JSON 형태의 위치 데이터

    @JsonProperty("detectionData")
    private String detectionData; // JSON 형태의 탐지 데이터
    
    @JsonProperty("createdAt")
    private LocalDateTime createdAt;

    @JsonProperty("evidenceFiles")
    private List<EvidenceFileResponse> evidenceFiles; // 증거 파일 목록 (선택적)

    /**
     * Event 엔티티로부터 EventResponse 생성 (증거 파일 없이)
     */
    public static EventResponse from(Event event) {
        return EventResponse.builder()
                .id(event.getId())
                .eventType(event.getEventType())
                .robotId(event.getRobotId())
                .severity(event.getSeverity())
                .locationData(event.getLocationData())
                .detectionData(event.getDetectionData())
                .createdAt(event.getCreatedAt())
                .build();
    }

    /**
     * Event 엔티티와 증거 파일로부터 EventResponse 생성
     */
    public static EventResponse from(Event event, List<EvidenceFileResponse> evidenceFiles) {
        return EventResponse.builder()
                .id(event.getId())
                .eventType(event.getEventType())
                .robotId(event.getRobotId())
                .severity(event.getSeverity())
                .locationData(event.getLocationData())
                .detectionData(event.getDetectionData())
                .createdAt(event.getCreatedAt())
                .evidenceFiles(evidenceFiles)
                .build();
    }
}