package com.ssafy.afterschool.domain.user.controller;

import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.service.UserService;
import com.ssafy.afterschool.global.constants.SuccessCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.security.UserPrincipal;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.security.SecurityRequirement;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequestMapping("/users")
@RequiredArgsConstructor
@Tag(name = "User", description = "사용자 API")
@SecurityRequirement(name = "Bearer Authentication")
public class UserController {

    private final UserService userService;

    @Operation(summary = "내 프로필 조회", description = "현재 로그인한 사용자의 프로필 정보를 조회합니다.")
    @GetMapping("/me")
    public ApiResponse<UserInfoResponse> getMyProfile(@AuthenticationPrincipal UserPrincipal userPrincipal) {
        UserInfoResponse response = userService.getMyProfile(userPrincipal.getId());
        return ApiResponse.success(SuccessCode.USER_PROFILE_SUCCESS, response);
    }
}