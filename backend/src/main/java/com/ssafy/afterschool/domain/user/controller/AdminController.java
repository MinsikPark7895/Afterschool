package com.ssafy.afterschool.domain.user.controller;

import com.ssafy.afterschool.domain.user.dto.UserCreateRequest;
import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.dto.UserUpdateRequest;
import com.ssafy.afterschool.domain.user.service.UserService;
import com.ssafy.afterschool.global.constants.SuccessCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.response.PageResponse;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.security.SecurityRequirement;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
import org.springframework.data.web.PageableDefault;
import org.springframework.security.access.prepost.PreAuthorize;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequestMapping("/admin")
@RequiredArgsConstructor
@Tag(name = "User Admin", description = "사용자 관리 API (관리자 전용)")
@SecurityRequirement(name = "Bearer Authentication")
@PreAuthorize("hasAuthority('ADMIN')")
public class AdminController {

    private final UserService userService;

    @Operation(summary = "사용자 목록 조회", description = "관리자 권한으로 모든 사용자 목록을 조회합니다.")
    @GetMapping("/users")
    public ApiResponse<PageResponse<UserInfoResponse>> getAllUsers(
            @PageableDefault(size = 20, sort = "createdAt", direction = Sort.Direction.DESC) Pageable pageable) {
        Page<UserInfoResponse> page = userService.getAllUsers(pageable);
        PageResponse<UserInfoResponse> response = PageResponse.of(page);
        return ApiResponse.success(SuccessCode.USER_LIST_SUCCESS, response);
    }

    @Operation(summary = "사용자 상세 조회", description = "관리자 권한으로 특정 사용자 정보를 조회합니다.")
    @GetMapping("/{userId}")
    public ApiResponse<UserInfoResponse> getUserById(
            @Parameter(description = "사용자 ID", example = "1") @PathVariable Long userId) {
        UserInfoResponse response = userService.getUserById(userId);
        return ApiResponse.success(SuccessCode.USER_PROFILE_SUCCESS, response);
    }

    @Operation(summary = "사용자 생성", description = "관리자 권한으로 새로운 사용자를 생성합니다.")
    @PostMapping("/create-user")
    public ApiResponse<UserInfoResponse> createUser(@Valid @RequestBody UserCreateRequest request) {
        UserInfoResponse response = userService.createUser(request);
        return ApiResponse.success(SuccessCode.USER_CREATE_SUCCESS, response);
    }

    @Operation(summary = "사용자 정보 수정", description = "관리자 권한으로 사용자 정보를 수정합니다.")
    @PutMapping("/update-user/{userId}")
    public ApiResponse<UserInfoResponse> updateUser(
            @Parameter(description = "사용자 ID", example = "1") @PathVariable Long userId,
            @Valid @RequestBody UserUpdateRequest request) {
        UserInfoResponse response = userService.updateUser(userId, request);
        return ApiResponse.success(SuccessCode.USER_UPDATE_SUCCESS, response);
    }

    @Operation(summary = "사용자 삭제", description = "관리자 권한으로 사용자를 소프트 삭제합니다.")
    @DeleteMapping("/delete-user/{userId}")
    public ApiResponse<Void> deleteUser(
            @Parameter(description = "사용자 ID", example = "1") @PathVariable Long userId) {
        userService.deleteUser(userId);
        return ApiResponse.success(SuccessCode.USER_DELETE_SUCCESS);
    }
}