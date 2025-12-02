package com.ssafy.afterschool.domain.event.entity;

/**
 * 명령 상태 열거형
 */
public enum CommandStatus {
    SENT("전송됨"),
    SUCCESS("성공"),
    FAILED("실패");

    private final String description;

    CommandStatus(String description) {
        this.description = description;
    }

    public String getDescription() {
        return description;
    }
}