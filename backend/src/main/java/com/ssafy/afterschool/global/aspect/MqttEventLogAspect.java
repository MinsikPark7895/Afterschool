package com.ssafy.afterschool.global.aspect;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.ssafy.afterschool.domain.event.entity.Event;
import com.ssafy.afterschool.domain.event.service.EventLogService;
import com.ssafy.afterschool.global.mqtt.MqttMessageRouter.*;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;
import org.springframework.stereotype.Component;

/**
 * MQTT 이벤트 로그 자동 기록 AOP
 * - @LogMqttEvent 어노테이션이 붙은 메서드의 MQTT 이벤트를 자동으로 Event 테이블에 기록
 * - 증거 파일 처리 옵션 지원
 */
@Slf4j
@Aspect
@Component
@RequiredArgsConstructor
public class MqttEventLogAspect {

    private final EventLogService eventLogService;
    private final ObjectMapper objectMapper;

    @Around("@annotation(logMqttEvent)")
    public Object logEvent(ProceedingJoinPoint joinPoint, LogMqttEvent logMqttEvent) throws Throwable {
        if (!logMqttEvent.enabled()) {
            return joinPoint.proceed();
        }

        Object[] args = joinPoint.getArgs();
        Object mqttEventObj = extractMqttEvent(args);
        
        if (mqttEventObj == null) {
            log.warn("⚠️ MQTT 이벤트 객체를 찾을 수 없습니다");
            return joinPoint.proceed();
        }

        try {
            // 1. MQTT 이벤트에서 데이터 추출
            String topic = extractTopic(mqttEventObj);
            String payload = extractPayload(mqttEventObj);
            String robotId = extractRobotId(payload);
            
            log.info("📝 MQTT 이벤트 로그 기록 시작 - Type: {}, Robot: {}", 
                logMqttEvent.eventType(), robotId);

            // 2. 실제 이벤트 처리 메서드 실행
            Object result = joinPoint.proceed();

            // 3. 이벤트 데이터 파싱 및 추출
            JsonNode messageNode = objectMapper.readTree(payload);
            JsonNode header = messageNode.get("header");
            JsonNode payloadNode = messageNode.get("payload");

            String severity = extractSeverity(payloadNode, logMqttEvent.defaultSeverity());
            String locationData = extractLocationData(payloadNode);
            String eventData = extractEventData(payloadNode, logMqttEvent.eventType());

            // 4. 이벤트 저장
            Event savedEvent = eventLogService.createEvent(
                logMqttEvent.eventType(), robotId, severity, locationData, eventData
            );

            log.info("✅ MQTT 이벤트 로그 기록 완료 - EventID: {}, Type: {}, Robot: {}", 
                savedEvent.getId(), logMqttEvent.eventType(), robotId);

            // 5. 증거 파일 처리 (필요한 경우)
            if (logMqttEvent.processEvidenceFiles()) {
                processEvidenceFiles(savedEvent.getId(), payloadNode);
            }

            return result;

        } catch (Exception e) {
            log.error("❌ MQTT 이벤트 로그 기록 실패 - Type: {}", logMqttEvent.eventType(), e);
            // 이벤트 처리는 계속 진행 (로그 기록 실패가 전체 프로세스를 방해하면 안됨)
            return joinPoint.proceed();
        }
    }

    /**
     * 메서드 인자에서 MQTT 이벤트 객체 추출
     */
    private Object extractMqttEvent(Object[] args) {
        for (Object arg : args) {
            if (arg instanceof DetectionMqttEvent || 
                arg instanceof MissionMqttEvent || 
                arg instanceof SystemMqttEvent ||
                arg instanceof RobotMqttEvent) {
                return arg;
            }
        }
        return null;
    }

    /**
     * MQTT 이벤트에서 토픽 추출
     */
    private String extractTopic(Object mqttEvent) {
        if (mqttEvent instanceof DetectionMqttEvent event) return event.topic();
        if (mqttEvent instanceof MissionMqttEvent event) return event.topic();
        if (mqttEvent instanceof SystemMqttEvent event) return event.topic();
        if (mqttEvent instanceof RobotMqttEvent event) return event.topic();
        return "unknown";
    }

    /**
     * MQTT 이벤트에서 페이로드 추출
     */
    private String extractPayload(Object mqttEvent) {
        if (mqttEvent instanceof DetectionMqttEvent event) return event.payload();
        if (mqttEvent instanceof MissionMqttEvent event) return event.payload();
        if (mqttEvent instanceof SystemMqttEvent event) return event.payload();
        if (mqttEvent instanceof RobotMqttEvent event) return event.payload();
        return "{}";
    }

    /**
     * 페이로드에서 로봇 ID 추출
     */
    private String extractRobotId(String payload) {
        try {
            JsonNode messageNode = objectMapper.readTree(payload);
            JsonNode header = messageNode.get("header");
            
            if (header != null && header.has("robot_id")) {
                return header.get("robot_id").asText();
            }
        } catch (Exception e) {
            log.warn("⚠️ 로봇 ID 추출 실패", e);
        }
        return "unknown";
    }

    /**
     * 페이로드에서 심각도 추출
     */
    private String extractSeverity(JsonNode payloadNode, String defaultSeverity) {
        if (payloadNode != null && payloadNode.has("severity")) {
            return payloadNode.get("severity").asText();
        }
        return defaultSeverity;
    }

    /**
     * 위치 데이터 추출
     */
    private String extractLocationData(JsonNode payloadNode) {
        try {
            if (payloadNode != null && payloadNode.has("location")) {
                return objectMapper.writeValueAsString(payloadNode.get("location"));
            }
        } catch (Exception e) {
            log.warn("⚠️ 위치 데이터 추출 실패", e);
        }
        return null;
    }

    /**
     * 이벤트별 데이터 추출
     */
    private String extractEventData(JsonNode payloadNode, String eventType) {
        try {
            if (payloadNode == null) return null;

            return switch (eventType) {
                case "detection" -> extractDetectionData(payloadNode);
                case "mission_done" -> extractMissionData(payloadNode);
                case "system_status" -> extractSystemData(payloadNode);
                default -> objectMapper.writeValueAsString(payloadNode);
            };
        } catch (Exception e) {
            log.warn("⚠️ 이벤트 데이터 추출 실패 - Type: {}", eventType, e);
            return null;
        }
    }

    /**
     * 탐지 이벤트 데이터 추출
     */
    private String extractDetectionData(JsonNode payloadNode) throws Exception {
        ObjectNode detectionInfo = objectMapper.createObjectNode();
        detectionInfo.set("detection_info", payloadNode.get("detection_info"));
        detectionInfo.set("threat_assessment", payloadNode.get("threat_assessment"));
        detectionInfo.set("evidence", payloadNode.get("evidence"));
        detectionInfo.set("cooperation_request", payloadNode.get("cooperation_request"));
        return objectMapper.writeValueAsString(detectionInfo);
    }

    /**
     * 미션 완료 데이터 추출
     */
    private String extractMissionData(JsonNode payloadNode) throws Exception {
        return objectMapper.writeValueAsString(payloadNode);
    }

    /**
     * 시스템 상태 데이터 추출
     */
    private String extractSystemData(JsonNode payloadNode) throws Exception {
        return objectMapper.writeValueAsString(payloadNode);
    }

    /**
     * 증거 파일 처리
     */
    private void processEvidenceFiles(Long eventId, JsonNode payloadNode) {
        try {
            if (payloadNode == null || !payloadNode.has("evidence")) {
                return;
            }

            JsonNode evidence = payloadNode.get("evidence");
            if (evidence.has("image_path")) {
                String imagePath = evidence.get("image_path").asText();
                String s3Key = extractS3KeyFromPath(imagePath);
                
                if (s3Key != null) {
                    eventLogService.createEvidenceFile(
                        eventId,
                        "afterschool-evidence", // TODO: 설정으로 분리
                        s3Key,
                        extractFilenameFromPath(imagePath),
                        0L, // TODO: 파일 크기 처리
                        "image/jpeg",
                        null
                    );
                    
                    log.info("📁 증거 파일 등록 완료 - EventID: {}, S3Key: {}", eventId, s3Key);
                }
            }
        } catch (Exception e) {
            log.error("❌ 증거 파일 처리 실패 - EventID: {}", eventId, e);
        }
    }

    /**
     * 파일 경로에서 S3 키 추출
     */
    private String extractS3KeyFromPath(String filePath) {
        if (filePath == null || !filePath.startsWith("/evidence/")) {
            return null;
        }
        
        String filename = filePath.substring(10); // "/evidence/" 제거
        
        // 날짜 정보로 폴더 구조 생성
        if (filename.length() > 20) {
            String datePart = filename.substring(filename.length() - 19, filename.length() - 7);
            if (datePart.matches("\\d{8}")) {
                String year = datePart.substring(0, 4);
                String month = datePart.substring(4, 6);  
                String day = datePart.substring(6, 8);
                return String.format("evidence/%s/%s/%s/%s", year, month, day, filename);
            }
        }
        
        return "evidence/" + filename;
    }

    /**
     * 파일 경로에서 파일명 추출
     */
    private String extractFilenameFromPath(String filePath) {
        if (filePath == null) return "unknown_file";
        
        int lastSlash = filePath.lastIndexOf('/');
        return lastSlash >= 0 ? filePath.substring(lastSlash + 1) : filePath;
    }
}
