package com.ssafy.afterschool.domain.auth.controller;

import com.ssafy.afterschool.domain.auth.dto.LoginRequest;
import com.ssafy.afterschool.domain.auth.dto.LoginResponse;
import com.ssafy.afterschool.domain.auth.dto.LogoutRequest;
import com.ssafy.afterschool.domain.auth.service.AuthService;
import com.ssafy.afterschool.global.constants.SuccessCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.security.UserPrincipal;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.web.bind.annotation.*;

/**
 * 인증 컨트롤러
 */
@Slf4j
@RestController
@RequestMapping("/auth")
@RequiredArgsConstructor
@Tag(name = "Authentication", description = "인증 관련 API")
public class AuthController {

    private final AuthService authService;

    @Operation(summary = "로그인", description = "사용자 로그인 후 JWT 토큰을 발급합니다.")
    @PostMapping("/login")
    public ApiResponse<LoginResponse> login(@Valid @RequestBody LoginRequest request) {
        LoginResponse response = authService.login(request);
        return ApiResponse.success(SuccessCode.LOGIN_SUCCESS, response);
    }

    @Operation(summary = "로그아웃", description = "리프레시 토큰을 블랙리스트에 추가하여 로그아웃을 처리합니다.")
    @PostMapping("/logout")
    public ApiResponse<Void> logout(@Valid @RequestBody LogoutRequest request) {
        authService.logout(request.getRefreshToken());
        return ApiResponse.success(SuccessCode.LOGOUT_SUCCESS);
    }

    @Operation(summary = "토큰 갱신", description = "리프레시 토큰으로 새로운 액세스 토큰과 리프레시 토큰을 발급합니다.")
    @PostMapping("/refresh")
    public ApiResponse<LoginResponse> refreshToken(@RequestBody LogoutRequest request) {
        LoginResponse response = authService.refreshToken(request.getRefreshToken());
        return ApiResponse.success(SuccessCode.TOKEN_REFRESH_SUCCESS, response);
    }
}