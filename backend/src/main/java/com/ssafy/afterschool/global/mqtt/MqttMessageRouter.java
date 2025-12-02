package com.ssafy.afterschool.global.mqtt;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.coordinate.service.CoordinateTransformService;
import com.ssafy.afterschool.global.coordinate.dto.Position;
import com.ssafy.afterschool.global.exception.ApiException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.ApplicationEventPublisher;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.Message;
import org.springframework.stereotype.Component;

/**
 * MQTT 메시지 라우터
 * - 수신한 MQTT 메시지를 토픽별로 적절한 도메인으로 분배
 * - 각 도메인이 Spring Event로 처리할 수 있도록 전달
 */
@Slf4j
@Component
@RequiredArgsConstructor
public class MqttMessageRouter {

    private final ApplicationEventPublisher eventPublisher;
    private final ObjectMapper objectMapper;
    private final CoordinateTransformService coordinateTransformService;

    /**
     * MQTT 메시지 수신 및 라우팅
     * - JSON 파싱, 좌표 변환, 도메인 이벤트 발행
     */
    @ServiceActivator(inputChannel = "mqttInputChannel")
    public void routeMessage(Message<?> message) {
        String topic = null;
        try {
            topic = (String) message.getHeaders().get("mqtt_receivedTopic");
            String payload = message.getPayload().toString();

            if (topic == null || topic.trim().isEmpty()) {
                log.error("❌ MQTT 토픽이 누락되었습니다");
                return;
            }

            log.info("📥 MQTT 메시지 라우팅 - Topic: {}", topic);
            log.debug("📥 MQTT 페이로드: {}", payload);

            // 1. JSON 파싱 및 검증
            JsonNode messageNode = parseAndValidateMessage(payload);
            if (messageNode == null) {
                return;
            }

            // 2. 좌표 변환 (로봇 상태 메시지에만 적용)
            JsonNode transformedMessage = messageNode;
            if (topic.contains("/status/")) {
                transformedMessage = transformCoordinatesIfNeeded(messageNode);
                log.debug("🔄 좌표 변환 적용: {}", topic);
            }

            // 3. 변환된 메시지로 도메인 이벤트 발행
            String transformedPayload = objectMapper.writeValueAsString(transformedMessage);
            publishDomainEvent(topic, transformedPayload);

        } catch (Exception e) {
            log.error("❌ MQTT 메시지 라우팅 중 오류 발생 - Topic: {}, Error: {}", topic, e.getMessage(), e);
            // MQTT 메시지 처리는 실패해도 계속 진행 (안정성 우선)
        }
    }
    
    /**
     * JSON 메시지 파싱 및 검증
     */
    private JsonNode parseAndValidateMessage(String payload) {
        try {
            if (payload == null || payload.trim().isEmpty()) {
                log.warn("⚠️ MQTT 페이로드가 비어있습니다");
                return null;
            }

            JsonNode messageNode = objectMapper.readTree(payload);

            // 기본 구조 검증
            if (!messageNode.has("header") || !messageNode.has("payload")) {
                log.warn("⚠️ 잘못된 MQTT 메시지 구조 - header 또는 payload 누락. Message: {}", payload.length() > 200 ? payload.substring(0, 200) + "..." : payload);
                return null;
            }

            return messageNode;

        } catch (Exception e) {
            log.error("❌ MQTT 메시지 파싱 실패 - Payload length: {}, Error: {}",
                payload != null ? payload.length() : 0, e.getMessage());
            return null;
        }
    }
    
    /**
     * 좌표 변환 처리 (필요한 경우에만)
     */
    private JsonNode transformCoordinatesIfNeeded(JsonNode messageNode) {
        try {
            JsonNode payload = messageNode.get("payload");
            
            // 위치 정보가 있는지 확인
            if (payload.has("position")) {
                JsonNode position = payload.get("position");
                
                double rosX = position.get("x").asDouble();
                double rosY = position.get("y").asDouble();
                double rosZ = position.get("z").asDouble(0.0);
                
                // ROS 좌표 → 픽셀 좌표 변환
                Position rosPosition = Position.builder()
                        .x(rosX)
                        .y(rosY)
                        .z(rosZ)
                        .build();

                Position pixelCoord = coordinateTransformService.rosToPixel(rosPosition);

                log.debug("🔄 좌표 변환 - ROS({}, {}) → 픽셀({}, {})",
                    rosX, rosY, pixelCoord.getX(), pixelCoord.getY());

                // 변환된 좌표 추가 (원본 보존)
                JsonNode mutableNode = messageNode.deepCopy();
                JsonNode mutablePosition = mutableNode.get("payload").get("position");
                
                ((com.fasterxml.jackson.databind.node.ObjectNode) mutablePosition)
                    .put("pixel_x", pixelCoord.getX())
                    .put("pixel_y", pixelCoord.getY());
                
                return mutableNode;
            }
            
            return messageNode;
            
        } catch (Exception e) {
            log.error("❌ 좌표 변환 중 오류", e);
            return messageNode;
        }
    }
    
    /**
     * 도메인별 이벤트 발행 (정확한 토픽 매칭)
     */
    private void publishDomainEvent(String topic, String payload) {
        log.debug("🔍 토픽 매칭 시도: {}", topic);

        // 로봇 상태 메시지 (basic/detail)
        if (topic.contains("/status/")) {
            eventPublisher.publishEvent(new RobotMqttEvent(topic, payload));
            log.debug("✓ RobotMqttEvent 발행: {}", topic);
        }
        // 침입자 탐지 이벤트
        else if (topic.contains("/detection/")) {
            eventPublisher.publishEvent(new DetectionMqttEvent(topic, payload));
            log.debug("✓ DetectionMqttEvent 발행: {}", topic);
        }
        // 임무 완료 이벤트 (mission_done 정확한 매칭)
        else if (topic.contains("/mission_done/")) {
            eventPublisher.publishEvent(new MissionMqttEvent(topic, payload));
            log.debug("✓ MissionMqttEvent 발행: {}", topic);
        }
        // 시스템 상태
        else if (topic.contains("/system/")) {
            eventPublisher.publishEvent(new SystemMqttEvent(topic, payload));
            log.debug("✓ SystemMqttEvent 발행: {}", topic);
        }
        else {
            log.warn("⚠️ 알 수 없는 토픽: {} (지원되는 패턴: /status/, /detection/, /mission_done/, /system/)", topic);
        }
    }
    
    // 도메인 이벤트 클래스들
    public record RobotMqttEvent(String topic, String payload) {}
    public record DetectionMqttEvent(String topic, String payload) {}
    public record MissionMqttEvent(String topic, String payload) {}
    public record SystemMqttEvent(String topic, String payload) {}
}
