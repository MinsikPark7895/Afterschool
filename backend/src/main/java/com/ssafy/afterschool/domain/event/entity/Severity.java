package com.ssafy.afterschool.domain.event.entity;

/**
 * 이벤트 심각도 열거형
 */
public enum Severity {
    CRITICAL("심각"),
    WARNING("경고"),
    INFO("정보");

    private final String description;

    Severity(String description) {
        this.description = description;
    }

    public String getDescription() {
        return description;
    }
}