package com.ssafy.afterschool.global.mqtt;

import lombok.experimental.UtilityClass;

/**
 * MQTT 토픽 유틸리티 클래스
 * - MQTT 토픽 전용 처리
 * - 토픽 파싱 및 검증 로직
 */
@UtilityClass
public class MqttUtils {

    /**
     * MQTT 토픽에서 로봇 ID 추출
     * - from_robot/status/basic/tb1 → tb1
     * - from_robot/event/detection/tb2 → tb2
     */
    public static String extractRobotIdFromTopic(String topic) {
        if (topic == null || topic.trim().isEmpty()) {
            return null;
        }

        String[] parts = topic.split("/");
        if (parts.length >= 4 && "from_robot".equals(parts[0])) {
            return parts[3]; // 마지막 부분이 robot_id
        }

        return null;
    }

    /**
     * 토픽 유효성 검증
     */
    public static boolean isValidTopic(String topic) {
        return topic != null && !topic.trim().isEmpty();
    }


}
