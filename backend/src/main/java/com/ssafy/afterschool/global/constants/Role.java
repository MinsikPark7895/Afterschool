package com.ssafy.afterschool.global.constants;

import lombok.AllArgsConstructor;
import lombok.Getter;

/**
 * 사용자 권한 열거형
 */
@Getter
@AllArgsConstructor
public enum Role {
    ADMIN("관리자", "ROLE_ADMIN"),
    OPERATOR("운영자", "ROLE_OPERATOR");

    private final String description;
    private final String authority;
}