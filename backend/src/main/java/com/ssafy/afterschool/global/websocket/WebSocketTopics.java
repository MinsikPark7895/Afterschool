package com.ssafy.afterschool.global.websocket;

import lombok.experimental.UtilityClass;

/**
 * WebSocket STOMP 토픽 패턴 상수 클래스
 * - 토픽 상수 정의 및 빌더 메서드만 포함
 * - 유틸리티 기능은 WebSocketUtils에서 처리
 */
@UtilityClass
public class WebSocketTopics {

    // ======================== 토픽 베이스 ========================

    /**
     * WebSocket 토픽 루트
     */
    public static final String TOPIC_BASE = "/topic/";

    /**
     * 로봇 관련 토픽 베이스
     */
    public static final String ROBOT_BASE = TOPIC_BASE + "robot/";

    /**
     * 이벤트 관련 토픽 베이스
     */
    public static final String EVENT_BASE = TOPIC_BASE + "event/";

    /**
     * 명령 관련 토픽 베이스
     */
    public static final String COMMAND_BASE = TOPIC_BASE + "command/";

    /**
     * 시스템 관련 토픽 베이스
     */
    public static final String SYSTEM_BASE = TOPIC_BASE + "system/";

    // ======================== 토픽 빌더 메서드 ========================

    /**
     * 로봇 토픽 생성
     */
    public static String robot(String category, String robotId) {
        return robotId != null ? ROBOT_BASE + category + "/" + robotId : ROBOT_BASE + category;
    }

    public static String robot(String category) {
        return ROBOT_BASE + category;
    }

    /**
     * 이벤트 토픽 생성
     */
    public static String event(String category, String robotId) {
        return robotId != null ? EVENT_BASE + category + "/" + robotId : EVENT_BASE + category;
    }

    public static String event(String category) {
        return EVENT_BASE + category;
    }

    /**
     * 명령 토픽 생성
     */
    public static String command(String category, String robotId) {
        return robotId != null ? COMMAND_BASE + category + "/" + robotId : COMMAND_BASE + category;
    }

    public static String command(String category) {
        return COMMAND_BASE + category;
    }

    /**
     * 시스템 토픽 생성
     */
    public static String system(String category) {
        return SYSTEM_BASE + category;
    }
}