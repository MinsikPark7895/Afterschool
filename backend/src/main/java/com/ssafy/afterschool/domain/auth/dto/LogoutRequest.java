package com.ssafy.afterschool.domain.auth.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.NoArgsConstructor;

/**
 * 로그아웃 요청 DTO
 */
@Getter
@NoArgsConstructor
public class LogoutRequest {

    @NotBlank(message = "리프레시 토큰은 필수입니다")
    private String refreshToken;
}