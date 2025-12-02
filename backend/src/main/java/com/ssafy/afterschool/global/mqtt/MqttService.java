package com.ssafy.afterschool.global.mqtt;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.afterschool.domain.robot.dto.CommandResponse;
import com.ssafy.afterschool.domain.robot.dto.MoveToRequest;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.coordinate.dto.Position;
import com.ssafy.afterschool.global.exception.ApiException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.integration.mqtt.support.MqttHeaders;
import org.springframework.integration.support.MessageBuilder;
import org.springframework.messaging.MessageChannel;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

/**
 * 통합 MQTT 서비스
 * - 모든 MQTT 통신을 단일 진입점으로 관리
 * - 표준화된 메시지 포맷 및 전송 로직 제공
 */
@Slf4j
@Service
@RequiredArgsConstructor
public class MqttService {

    private final MessageChannel mqttOutputChannel;
    private final ObjectMapper objectMapper;
    private final AtomicLong messageIdCounter = new AtomicLong(1);

    // ======================== 로봇 명령 전송 ========================

    /**
     * 로봇 이동 명령 전송
     */
    public CommandResponse sendMoveCommand(String robotId, Position rosPosition, MoveToRequest request) {
        String messageId = generateMessageId();
        String topic = MqttTopics.robotCommand(CommandType.MOVE_TO, robotId);

        MqttMessage.Payload payload = MqttMessage.Payload.builder()
            .commandType(CommandType.MOVE_TO)
            .data(Map.of(
                "target_position", Map.of(
                    "x", rosPosition.getX(),
                    "y", rosPosition.getY(),
                    "z", 0.0,
                    "orientation", Map.of("x", 0.0, "y", 0.0, "z", 0.0, "w", 1.0)
                ),
                "response_required", true,
                "timeout", 60
            ))
            .build();

        MqttMessage message = createStandardMessage(messageId, robotId, payload);
        sendMessage(topic, message);

        log.info("📤 로봇 이동 명령 전송 완료 - Robot: {}, MessageId: {}", robotId, messageId);
        return CommandResponse.sent(CommandType.MOVE_TO, "이동 명령이 전송되었습니다");
    }

    /**
     * 순찰 시작 명령 전송 (일괄)
     */
    public CommandResponse sendStartPatrolCommand(List<String> activeRobots) {
        String messageId = generateMessageId();
        String topic = MqttTopics.robotCommand(CommandType.START_PATROL);

        MqttMessage.Payload payload = MqttMessage.Payload.builder()
            .commandType(CommandType.START_PATROL)
            .data(Map.of(
                "affected_robots", activeRobots,
                "response_required", true,
                "timeout", 30
            ))
            .build();

        MqttMessage message = createStandardMessage(messageId, "system", payload);
        sendMessage(topic, message);

        String resultMessage = String.format("순찰 시작 명령이 %d개 로봇에 전송되었습니다", activeRobots.size());
        log.info("📤 순찰 시작 명령 전송 완료 - Robots: {}, MessageId: {}", activeRobots.size(), messageId);
        return CommandResponse.sent(CommandType.START_PATROL, resultMessage);
    }

    /**
     * 순찰 중지 명령 전송 (일괄)
     */
    public CommandResponse sendStopPatrolCommand(List<String> activeRobots) {
        String messageId = generateMessageId();
        String topic = MqttTopics.robotCommand(CommandType.STOP_PATROL);

        MqttMessage.Payload payload = MqttMessage.Payload.builder()
            .commandType(CommandType.STOP_PATROL)
            .data(Map.of(
                "affected_robots", activeRobots,
                "response_required", true,
                "timeout", 10
            ))
            .build();

        MqttMessage message = createStandardMessage(messageId, "system", payload);
        sendMessage(topic, message);

        String resultMessage = String.format("순찰 중지 명령이 %d개 로봇에 전송되었습니다", activeRobots.size());
        log.info("📤 순찰 중지 명령 전송 완료 - Robots: {}, MessageId: {}", activeRobots.size(), messageId);
        return CommandResponse.sent(CommandType.STOP_PATROL, resultMessage);
    }

    // ======================== 표준 메시지 생성 ========================

    /**
     * 표준 MQTT 메시지 생성
     */
    private MqttMessage createStandardMessage(String messageId, String robotId, MqttMessage.Payload payload) {
        MqttMessage.Header header = MqttMessage.Header.builder()
            .timestamp(LocalDateTime.now().toString())
            .messageId(messageId)
            .robotId(robotId)
            .build();

        return MqttMessage.builder()
            .header(header)
            .payload(payload)
            .build();
    }

    /**
     * 메시지 ID 생성
     */
    private String generateMessageId() {
        return "msg_" + messageIdCounter.getAndIncrement();
    }

    // ======================== MQTT 전송 ========================

    /**
     * MQTT 메시지 전송 (내부용)
     */
    private void sendMessage(String topic, MqttMessage message) {
        try {
            validateInputs(topic, message);

            String jsonPayload = objectMapper.writeValueAsString(message);

            log.debug("📤 MQTT 메시지 전송 - Topic: {}", topic);
            log.debug("📤 MQTT 페이로드: {}", jsonPayload);

            org.springframework.messaging.Message<String> springMessage = MessageBuilder
                .withPayload(jsonPayload)
                .setHeader(MqttHeaders.TOPIC, topic)
                .setHeader(MqttHeaders.QOS, 1)
                .setHeader(MqttHeaders.RETAINED, false)
                .build();

            boolean sent = mqttOutputChannel.send(springMessage);
            if (!sent) {
                throw new ApiException(ErrorCode.MQTT_CHANNEL_UNAVAILABLE);
            }

            log.info("✅ MQTT 메시지 전송 완료 - Topic: {}", topic);

        } catch (ApiException e) {
            log.error("❌ MQTT 메시지 전송 실패 - Topic: {}", topic, e);
            throw e;
        } catch (Exception e) {
            log.error("❌ MQTT 메시지 전송 중 예상치 못한 오류 - Topic: {}", topic, e);
            throw new ApiException(ErrorCode.MQTT_PUBLISH_FAILED, e);
        }
    }

    /**
     * 입력값 검증
     */
    private void validateInputs(String topic, MqttMessage message) {
        if (topic == null || topic.trim().isEmpty()) {
            throw new ApiException(ErrorCode.MQTT_TOPIC_INVALID);
        }
        if (message == null) {
            throw new ApiException(ErrorCode.MQTT_PAYLOAD_INVALID);
        }
    }
}