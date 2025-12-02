package com.ssafy.afterschool.global.websocket;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;

/**
 * 통합 WebSocket 서비스
 * - 모든 WebSocket 메시지 전송을 단일 진입점으로 관리
 * - 중복 코드 제거 및 일관된 메시지 포맷 제공
 */
@Slf4j
@Service
@RequiredArgsConstructor
public class WebSocketService {

    private final SimpMessagingTemplate messagingTemplate;

    /**
     * 로봇 상태 업데이트 전송
     */
    public void sendRobotStatus(String category, String robotId, Object payload) {
        try {
            String topic = WebSocketTopics.robot(category, robotId);
            messagingTemplate.convertAndSend(topic, payload);

            log.debug("📡 로봇 상태 전송 - Topic: {}", topic);

        } catch (Exception e) {
            log.error("❌ 로봇 상태 전송 실패 - Category: {}, Robot: {}", category, robotId, e);
        }
    }

    /**
     * 이벤트 알림 전송
     */
    public void sendEventAlert(String category, Object payload) {
        try {
            String topic = WebSocketTopics.event(category);
            messagingTemplate.convertAndSend(topic, payload);

            log.info("🚨 이벤트 알림 전송 - Topic: {}", topic);

        } catch (Exception e) {
            log.error("❌ 이벤트 알림 전송 실패 - Category: {}", category, e);
        }
    }

    /**
     * 명령 실행 결과 전송 (통합)
     */
    public void sendCommandResult(String robotId, String commandType, String status, Object result) {
        try {
            CommandResultMessage message = new CommandResultMessage(
                robotId, commandType, status, result, LocalDateTime.now()
            );

            // 개별 로봇 구독자에게 전송
            if (!"system".equals(robotId)) {
                messagingTemplate.convertAndSend(WebSocketTopics.command("result", robotId), message);
            }

            // 전체 구독자에게 전송
            messagingTemplate.convertAndSend(WebSocketTopics.command("result"), message);

            log.info("📤 명령 결과 전송 - Robot: {}, Command: {}, Status: {}", robotId, commandType, status);

        } catch (Exception e) {
            log.error("❌ 명령 결과 전송 실패 - Robot: {}, Command: {}", robotId, commandType, e);
        }
    }

    /**
     * 시스템 상태 알림 전송
     */
    public void sendSystemStatus(Object payload) {
        try {
            String topic = WebSocketTopics.system("status");
            messagingTemplate.convertAndSend(topic, payload);

            log.info("🔧 시스템 상태 전송 - Topic: {}", topic);

        } catch (Exception e) {
            log.error("❌ 시스템 상태 전송 실패", e);
        }
    }

    /**
     * 명령 실행 결과 메시지 DTO
     */
    public record CommandResultMessage(
        String robotId,
        String commandType,
        String status,
        Object result,
        LocalDateTime timestamp
    ) {}
}