package com.ssafy.afterschool.domain.event.entity;

/**
 * 이벤트 유형 열거형
 */
public enum EventType {
    DETECTION("침입자 탐지"),
    MISSION_DONE("임무 완료");

    private final String description;

    EventType(String description) {
        this.description = description;
    }

    public String getDescription() {
        return description;
    }
}