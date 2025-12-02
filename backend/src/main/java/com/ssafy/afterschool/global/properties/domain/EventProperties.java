package com.ssafy.afterschool.global.properties.domain;

import com.ssafy.afterschool.global.properties.template.BaseS3Properties;
import lombok.Getter;
import lombok.Setter;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

import java.util.Map;
import java.util.Set;

/**
 * 이벤트 도메인 설정 프로퍼티
 * - 증거 파일 S3 경로 관리
 * - 이벤트 타입별 기본 설정
 */
@Component
@ConfigurationProperties(prefix = "domain.event")
@Getter
@Setter
public class EventProperties implements BaseS3Properties {

    /**
     * 증거 파일 S3 경로 패턴
     */
    private String evidenceS3Pattern = "evidence/{robotId}/{eventId}/{filename}";

    /**
     * 지원하는 파일 타입들
     */
    private Set<String> supportedFileTypes = Set.of("image", "video", "screenshot");

    /**
     * 이벤트 타입별 기본 심각도 매핑
     */
    private Map<String, String> defaultSeverity = Map.of(
            "detection", "critical",
            "mission_done", "info",
            "map_complete", "info"
    );

    /**
     * 파일 타입별 MIME 타입 매핑
     */
    private Map<String, String> mimeTypeMapping = Map.of(
            "image", "image/jpeg",
            "video", "video/mp4",
            "screenshot", "image/png"
    );

    /**
     * 알림 우선순위 매핑
     */
    private Map<String, String> notificationPriority = Map.of(
            "critical", "high",
            "warning", "normal",
            "info", "low"
    );

    // =====================================
    // BaseS3Properties 구현
    // =====================================

    @Override
    public String generateS3Key(String identifier) {
        // identifier 형식: "robotId/eventId/filename"
        String[] parts = identifier.split("/");
        if (parts.length != 3) {
            throw new IllegalArgumentException("증거 파일 S3 키 생성에는 'robotId/eventId/filename' 형식이 필요합니다");
        }

        String robotId = parts[0];
        String eventId = parts[1];
        String filename = parts[2];

        return evidenceS3Pattern
                .replace("{robotId}", robotId)
                .replace("{eventId}", eventId)
                .replace("{filename}", filename);
    }

    @Override
    public String getFileName(String identifier) {
        String[] parts = identifier.split("/");
        if (parts.length != 3) {
            throw new IllegalArgumentException("증거 파일 식별자 형식이 올바르지 않습니다");
        }
        return parts[2]; // filename 부분 반환
    }

    @Override
    public boolean isSupported(String fileType) {
        return !supportedFileTypes.contains(fileType);
    }

    // =====================================
    // 추가 유틸리티 메서드들
    // =====================================

    /**
     * 증거 파일 S3 키 생성 (편의 메서드)
     */
    public String generateEvidenceS3Key(String robotId, Long eventId, String filename) {
        return generateS3Key(robotId + "/" + eventId + "/" + filename);
    }

    /**
     * 이벤트 타입별 기본 심각도 반환
     */
    public String getDefaultSeverity(String eventType) {
        return defaultSeverity.getOrDefault(eventType, "info");
    }

    /**
     * 파일 타입별 MIME 타입 반환
     */
    public String getMimeType(String fileType) {
        return mimeTypeMapping.getOrDefault(fileType, "application/octet-stream");
    }

    /**
     * 심각도별 알림 우선순위 반환
     */
    public String getNotificationPriority(String severity) {
        return notificationPriority.getOrDefault(severity, "normal");
    }

    /**
     * 유효한 이벤트 타입인지 확인
     */
    public boolean isValidEventType(String eventType) {
        return defaultSeverity.containsKey(eventType);
    }
}
