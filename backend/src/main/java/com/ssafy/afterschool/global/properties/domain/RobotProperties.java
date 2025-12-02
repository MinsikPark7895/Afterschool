package com.ssafy.afterschool.global.properties.domain;

import com.ssafy.afterschool.global.properties.template.BaseMqttProperties;
import lombok.Getter;
import lombok.Setter;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

import java.util.Map;
import java.util.Set;

/**
 * 로봇 도메인 설정 프로퍼티
 * - MQTT 토픽 패턴 관리
 * - 로봇 목록 및 기본 설정
 */
@Component
@ConfigurationProperties(prefix = "domain.robot")
@Getter
@Setter
public class RobotProperties implements BaseMqttProperties {

    /**
     * 기본 로봇 ID 목록
     */
    private Set<String> robotIds = Set.of("tb1", "tb2");

    /**
     * MQTT 토픽 패턴
     * 예: from_robot/{robotId}/status/{type}
     */
    private String mqttTopicPattern = "{direction}_robot/{robotId}/{category}/{type}";

    /**
     * 로봇별 기본 이름
     */
    private Map<String, String> defaultNames = Map.of(
            "tb1", "순찰로봇 TB1",
            "tb2", "순찰로봇 TB2"
    );

    /**
     * MQTT 방향 타입
     */
    private Set<String> directions = Set.of("from", "to");

    /**
     * MQTT 카테고리 타입
     */
    private Set<String> categories = Set.of("status", "event", "command");

    // =====================================
    // BaseMqttProperties 구현
    // =====================================

    @Override
    public String generateTopic(String... params) {
        if (params.length != 4) {
            throw new IllegalArgumentException("MQTT 토픽 생성에는 4개 파라미터가 필요합니다: [direction, robotId, category, type]");
        }

        String direction = params[0];
        String robotId = params[1];
        String category = params[2];
        String type = params[3];

        // 유효성 검증
        if (!directions.contains(direction)) {
            throw new IllegalArgumentException("지원하지 않는 direction: " + direction);
        }
        if (!robotIds.contains(robotId)) {
            throw new IllegalArgumentException("지원하지 않는 robotId: " + robotId);
        }
        if (!categories.contains(category)) {
            throw new IllegalArgumentException("지원하지 않는 category: " + category);
        }

        return mqttTopicPattern
                .replace("{direction}", direction)
                .replace("{robotId}", robotId)
                .replace("{category}", category)
                .replace("{type}", type);
    }

    @Override
    public boolean isValidTopic(String topic) {
        if (topic == null || topic.trim().isEmpty()) {
            return false;
        }

        String[] parts = topic.split("/");
        if (parts.length != 4) {
            return false;
        }

        // from_robot 또는 to_robot 형태인지 확인
        String direction = parts[0].replace("_robot", "");
        String robotId = parts[1];
        String category = parts[2];

        return directions.contains(direction) && 
               robotIds.contains(robotId) && 
               categories.contains(category);
    }

    // =====================================
    // 추가 유틸리티 메서드들
    // =====================================

    /**
     * 로봇 이름 반환
     */
    public String getRobotName(String robotId) {
        return defaultNames.getOrDefault(robotId, robotId);
    }

    /**
     * 유효한 로봇 ID인지 확인
     */
    public boolean isValidRobotId(String robotId) {
        return robotIds.contains(robotId);
    }

    /**
     * from_robot 토픽 생성 (상태/이벤트 발행용)
     */
    public String generateFromRobotTopic(String robotId, String category, String type) {
        return generateTopic("from", robotId, category, type);
    }

    /**
     * to_robot 토픽 생성 (명령 전송용)
     */
    public String generateToRobotTopic(String robotId, String category, String type) {
        return generateTopic("to", robotId, category, type);
    }
}
