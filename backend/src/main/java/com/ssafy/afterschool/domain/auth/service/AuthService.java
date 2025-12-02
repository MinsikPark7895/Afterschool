package com.ssafy.afterschool.domain.auth.service;

import com.ssafy.afterschool.domain.auth.dto.LoginRequest;
import com.ssafy.afterschool.domain.auth.dto.LoginResponse;

/**
 * 인증 서비스 인터페이스
 */
public interface AuthService {

    /**
     * 로그인
     */
    LoginResponse login(LoginRequest request);

    /**
     * 로그아웃
     */
    void logout(String refreshToken);

    /**
     * 토큰 갱신
     */
    LoginResponse refreshToken(String refreshToken);
}