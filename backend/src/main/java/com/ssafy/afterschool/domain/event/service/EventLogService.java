package com.ssafy.afterschool.domain.event.service;

import com.ssafy.afterschool.domain.event.entity.Event;

/**
 * 이벤트 로깅 전용 서비스 인터페이스
 * - AOP에서 MQTT 이벤트를 자동으로 기록할 때 사용
 * - EventService와 책임 분리
 */
public interface EventLogService {

    /**
     * 이벤트 생성 (MQTT 메시지로부터)
     */
    Event createEvent(String eventType, String robotId, String severity,
                     String locationData, String detectionData);

    /**
     * 증거 파일 등록 (MQTT 메시지로부터)
     */
    void createEvidenceFile(Long eventId, String s3Bucket, String s3Key,
                           String originalFilename, Long fileSize, String mimeType, String metadata);
}