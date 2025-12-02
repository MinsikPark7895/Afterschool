package com.ssafy.afterschool.global.mqtt;

/**
 * 로봇 명령 타입 상수
 * - 어노테이션과 런타임 코드에서 모두 사용 가능한 컴파일 타임 상수
 * - 모든 명령 타입을 일관되게 관리
 */
public final class CommandType {

    public static final String MOVE_TO = "move_to";
    public static final String START_PATROL = "start_patrol";
    public static final String STOP_PATROL = "stop_patrol";

    private CommandType() {
        // 유틸리티 클래스 - 인스턴스화 방지
    }
}