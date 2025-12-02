package com.ssafy.afterschool.global.aspect;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * 로봇 명령 로그 기록 어노테이션
 * - RobotService의 메서드에 이 어노테이션을 붙이면 자동으로 CommandLog에 기록
 */
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface LogRobotCommand {
    /**
     * 명령 타입 (MOVE, START_PATROL, STOP_PATROL, EMERGENCY_STOP 등)
     */
    String commandType();
    
    /**
     * 로그 기록 여부 (기본값: true)
     */
    boolean enabled() default true;
}
