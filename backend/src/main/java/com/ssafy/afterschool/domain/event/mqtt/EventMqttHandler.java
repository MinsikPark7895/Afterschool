package com.ssafy.afterschool.domain.event.mqtt;

import com.ssafy.afterschool.domain.event.entity.EventType;
import com.ssafy.afterschool.global.aspect.LogMqttEvent;
import com.ssafy.afterschool.global.mqtt.MqttMessageRouter.DetectionMqttEvent;
import com.ssafy.afterschool.global.mqtt.MqttMessageRouter.MissionMqttEvent;
import com.ssafy.afterschool.global.mqtt.MqttMessageRouter.SystemMqttEvent;
import com.ssafy.afterschool.global.mqtt.MqttUtils;
import com.ssafy.afterschool.global.websocket.WebSocketService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.event.EventListener;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Component;

/**
 * 이벤트 도메인 MQTT 메시지 처리기
 * - 이벤트 도메인: 탐지/임무/시스템 이벤트 전용
 * - 역할: DB 저장 (AOP) + WebSocket 알림
 */
@Slf4j
@Component
@RequiredArgsConstructor
public class EventMqttHandler {

    private final WebSocketService webSocketService;

    /**
     * 침입자 탐지 이벤트 처리
     * - @LogMqttEvent AOP가 자동으로 Event 테이블에 저장
     * - processEvidenceFiles = true로 증거 파일도 자동 처리
     */
    @Async
    @EventListener
    @LogMqttEvent(
        eventType = "DETECTION",
        defaultSeverity = "critical",
        processEvidenceFiles = true
    )
    public void handleDetectionEvent(DetectionMqttEvent event) {
        try {
            log.info("🚨 침입자 탐지 이벤트 처리 - Topic: {}", event.topic());

            // DB 저장은 AOP가 자동 처리
            // WebSocket 긴급 알림 전송
            webSocketService.sendEventAlert("alert", event.payload());

            log.info("✅ 침입자 탐지 이벤트 처리 완료");
            
        } catch (Exception e) {
            log.error("❌ 침입자 탐지 이벤트 처리 실패", e);
        }
    }

    /**
     * 임무 완료 이벤트 처리
     * - @LogMqttEvent AOP가 자동으로 Event 테이블에 저장
     */
    @Async
    @EventListener
    @LogMqttEvent(
        eventType = "MISSION_DONE",
        defaultSeverity = "info"
    )
    public void handleMissionEvent(MissionMqttEvent event) {
        try {
            log.info("✅ 임무 완료 이벤트 처리 - Topic: {}", event.topic());

            String robotId = MqttUtils.extractRobotIdFromTopic(event.topic());

            // DB 저장은 AOP가 자동 처리
            // WebSocket 알림 전송 (임무 완료를 명령 결과로 통합)
            webSocketService.sendCommandResult(robotId, "mission_done", "success", event.payload());

            log.info("✅ 임무 완료 이벤트 처리 완료");
            
        } catch (Exception e) {
            log.error("❌ 임무 완료 이벤트 처리 실패", e);
        }
    }

    /**
     * 시스템 상태 이벤트 처리
     * - @LogMqttEvent AOP가 자동으로 Event 테이블에 저장
     */
    @Async
    @EventListener
    @LogMqttEvent(
        eventType = "system_status",
        defaultSeverity = "info"
    )
    public void handleSystemEvent(SystemMqttEvent event) {
        try {
            log.info("⚙️ 시스템 상태 이벤트 처리 - Topic: {}", event.topic());

            // DB 저장은 AOP가 자동 처리
            // WebSocket 알림 전송
            webSocketService.sendSystemStatus(event.payload());

            log.info("✅ 시스템 상태 이벤트 처리 완료");
            
        } catch (Exception e) {
            log.error("❌ 시스템 상태 이벤트 처리 실패", e);
        }
    }
}
