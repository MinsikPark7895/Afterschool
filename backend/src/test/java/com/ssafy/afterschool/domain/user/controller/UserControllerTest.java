package com.ssafy.afterschool.domain.user.controller;

import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.domain.user.service.UserService;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Role;
import com.ssafy.afterschool.global.constants.Status;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.security.UserPrincipal;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.time.LocalDateTime;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("UserController 단위 테스트")
class UserControllerTest {

    @Mock
    private UserService userService;

    @InjectMocks
    private UserController userController;

    private static final Long TEST_USER_ID = 1L;
    private static final String TEST_USERNAME = "testuser";
    private static final String TEST_NAME = "Test User";

    private UserInfoResponse userInfoResponse;
    private UserPrincipal userPrincipal;

    @BeforeEach
    void setUp() {
        userInfoResponse = UserInfoResponse.from(User.builder()
                .id(TEST_USER_ID)
                .username(TEST_USERNAME)
                .name(TEST_NAME)
                .role(Role.OPERATOR)
                .createdAt(LocalDateTime.now())
                .lastLoginAt(LocalDateTime.now())
                .build());

        userPrincipal = new UserPrincipal(TEST_USER_ID, TEST_USERNAME, "ROLE_OPERATOR");
    }

    @Test
    @DisplayName("내 프로필 조회가 성공한다")
    void shouldGetMyProfileSuccessfully() {
        // given
        given(userService.getMyProfile(TEST_USER_ID))
                .willReturn(userInfoResponse);

        // when
        ApiResponse<UserInfoResponse> response = userController.getMyProfile(userPrincipal);

        // then
        assertThat(response)
                .isNotNull()
                .satisfies(r -> {
                    assertThat(r.status()).isEqualTo(Status.SUCCESS);
                    assertThat(r.message()).isEqualTo("프로필 조회가 완료되었습니다");
                    assertThat(r.data()).isNotNull();
                    assertThat(r.data().getUserId()).isEqualTo(TEST_USER_ID);
                    assertThat(r.data().getUsername()).isEqualTo(TEST_USERNAME);
                    assertThat(r.data().getName()).isEqualTo(TEST_NAME);
                    assertThat(r.data().getRole()).isEqualTo(Role.OPERATOR);
                });

        then(userService).should().getMyProfile(TEST_USER_ID);
    }

    @Test
    @DisplayName("존재하지 않는 사용자의 프로필 조회 시 예외가 발생한다")
    void shouldThrowExceptionWhenUserNotFound() {
        // given
        given(userService.getMyProfile(TEST_USER_ID))
                .willThrow(new ApiException(ErrorCode.USER_NOT_FOUND));

        // when & then
        assertThatThrownBy(() -> userController.getMyProfile(userPrincipal))
                .isInstanceOf(ApiException.class)
                .satisfies(ex -> {
                    ApiException apiEx = (ApiException) ex;
                    assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                });

        then(userService).should().getMyProfile(TEST_USER_ID);
    }

    @Test
    @DisplayName("UserPrincipal에서 올바른 사용자 ID를 추출한다")
    void shouldExtractCorrectUserIdFromPrincipal() {
        // given
        given(userService.getMyProfile(TEST_USER_ID))
                .willReturn(userInfoResponse);

        // when
        userController.getMyProfile(userPrincipal);

        // then
        then(userService).should().getMyProfile(TEST_USER_ID);
    }

    @Test
    @DisplayName("서비스에서 반환된 응답이 올바르게 매핑된다")
    void shouldMapServiceResponseCorrectly() {
        // given
        given(userService.getMyProfile(TEST_USER_ID))
                .willReturn(userInfoResponse);

        // when
        ApiResponse<UserInfoResponse> response = userController.getMyProfile(userPrincipal);

        // then
        assertThat(response.data()).isEqualTo(userInfoResponse);
        then(userService).should().getMyProfile(TEST_USER_ID);
    }
}