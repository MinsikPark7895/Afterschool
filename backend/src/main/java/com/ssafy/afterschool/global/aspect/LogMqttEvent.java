package com.ssafy.afterschool.global.aspect;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * MQTT 이벤트 로그 기록 어노테이션
 * - MQTT 이벤트 핸들러 메서드에 이 어노테이션을 붙이면 자동으로 Event 테이블에 기록
 */
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface LogMqttEvent {
    /**
     * 이벤트 타입 (detection, mission_done, system_status 등)
     */
    String eventType();
    
    /**
     * 기본 심각도 (기본값: info)
     */
    String defaultSeverity() default "info";
    
    /**
     * 로그 기록 여부 (기본값: true)
     */
    boolean enabled() default true;
    
    /**
     * 증거 파일 처리 여부 (기본값: false)
     */
    boolean processEvidenceFiles() default false;
}
