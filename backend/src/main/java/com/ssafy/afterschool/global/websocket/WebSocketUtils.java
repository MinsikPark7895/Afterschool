package com.ssafy.afterschool.global.websocket;

import lombok.experimental.UtilityClass;

/**
 * WebSocket 토픽 유틸리티 클래스
 * - WebSocket 토픽 전용 처리
 * - 토픽 파싱 및 검증 로직
 */
@UtilityClass
public class WebSocketUtils {

    /**
     * WebSocket 토픽에서 로봇 ID 추출
     * - /topic/robot/basic/tb1 → tb1
     * - /topic/event/detection/tb2 → tb2
     */
    public static String extractRobotIdFromTopic(String topic) {
        if (topic == null || topic.trim().isEmpty()) {
            return null;
        }

        String[] parts = topic.split("/");
        // WebSocket 토픽 패턴: /topic/{type}/{category}/{robot_id}
        if (parts.length >= 5 && "topic".equals(parts[1])) {
            return parts[4]; // robot_id 위치
        }

        return null;
    }

    /**
     * WebSocket 토픽 유효성 검증
     */
    public static boolean isValidTopic(String topic) {
        return topic != null &&
               topic.startsWith("/topic/") &&
               !topic.trim().isEmpty();
    }

    /**
     * 토픽 타입 추출 (robot/event/command/system)
     */
    public static String extractTopicType(String topic) {
        if (topic == null) return null;

        String[] parts = topic.split("/");
        if (parts.length >= 3 && "topic".equals(parts[1])) {
            return parts[2]; // type 위치
        }

        return null;
    }

    /**
     * 토픽 카테고리 추출
     */
    public static String extractCategory(String topic) {
        if (topic == null) return null;

        String[] parts = topic.split("/");
        if (parts.length >= 4 && "topic".equals(parts[1])) {
            return parts[3]; // category 위치
        }

        return null;
    }
}