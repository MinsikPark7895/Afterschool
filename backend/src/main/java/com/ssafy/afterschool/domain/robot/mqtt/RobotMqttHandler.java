package com.ssafy.afterschool.domain.robot.mqtt;

import com.ssafy.afterschool.global.mqtt.MqttMessageRouter.RobotMqttEvent;
import com.ssafy.afterschool.global.websocket.WebSocketService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.event.EventListener;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Component;

/**
 * 로봇 상태 메시지 핸들러
 * - 로봇 도메인: 상태 업데이트 전용 (/status/)
 * - 역할: WebSocket 실시간 전송만
 * - 이벤트(탐지/임무/시스템)는 Event 도메인에서 처리
 */
@Slf4j
@Component
@RequiredArgsConstructor
public class RobotMqttHandler {

    private final WebSocketService webSocketService;

    /**
     * 로봇 상태 업데이트 처리 (로봇 도메인 전용)
     * - /status/basic/, /status/detail/ 토픽만 처리
     */
    @EventListener
    @Async("taskExecutor")
    public void handleRobotStatusUpdate(RobotMqttEvent event) {
        try {
            log.info("📥 로봇 상태 업데이트 - {}", event.topic());

            // MQTT 토픽에서 로봇 ID와 카테고리 추출
            String[] parts = event.topic().split("/");
            if (parts.length >= 4) {
                String category = parts[2]; // basic 또는 detail
                String robotId = parts[3];

                webSocketService.sendRobotStatus(category, robotId, event.payload());
            }

        } catch (Exception e) {
            log.error("❌ 로봇 상태 업데이트 실패 - {}", event.topic(), e);
        }
    }

    // 로봇 도메인은 상태 업데이트만 처리
    // 이벤트(탐지/임무/시스템)는 Event 도메인에서 처리
}