package com.ssafy.afterschool.domain.auth.controller;

import com.ssafy.afterschool.domain.auth.dto.LoginRequest;
import com.ssafy.afterschool.domain.auth.dto.LoginResponse;
import com.ssafy.afterschool.domain.auth.dto.LogoutRequest;
import com.ssafy.afterschool.domain.auth.service.AuthService;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Role;
import com.ssafy.afterschool.global.constants.Status;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.response.ApiResponse;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.test.util.ReflectionTestUtils;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("AuthController 단위 테스트")
class AuthControllerTest {

    @Mock
    private AuthService authService;

    @InjectMocks
    private AuthController authController;

    private static final String TEST_USERNAME = "testuser";
    private static final String TEST_PASSWORD = "password123";
    private static final String TEST_ACCESS_TOKEN = "access.token.jwt";
    private static final String TEST_REFRESH_TOKEN = "refresh.token.jwt";
    private static final Long TEST_USER_ID = 1L;

    private LoginRequest loginRequest;
    private LoginResponse loginResponse;
    private LogoutRequest logoutRequest;

    @BeforeEach
    void setUp() {
        loginRequest = LoginRequest.builder()
                .username(TEST_USERNAME)
                .password(TEST_PASSWORD)
                .build();

        loginResponse = LoginResponse.builder()
                .accessToken(TEST_ACCESS_TOKEN)
                .refreshToken(TEST_REFRESH_TOKEN)
                .userId(TEST_USER_ID)
                .username(TEST_USERNAME)
                .role(Role.OPERATOR.name())
                .build();

        logoutRequest = new LogoutRequest();
        ReflectionTestUtils.setField(logoutRequest, "refreshToken", TEST_REFRESH_TOKEN);
    }

    @Nested
    @DisplayName("로그인 테스트")
    class LoginTest {

        @Test
        @DisplayName("유효한 로그인 정보로 로그인이 성공한다")
        void shouldLoginSuccessfullyWithValidCredentials() {
            // given
            given(authService.login(any(LoginRequest.class)))
                    .willReturn(loginResponse);

            // when
            ApiResponse<LoginResponse> response = authController.login(loginRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("로그인이 성공했습니다");
                        assertThat(r.data()).isNotNull();
                        assertThat(r.data().getAccessToken()).isEqualTo(TEST_ACCESS_TOKEN);
                        assertThat(r.data().getRefreshToken()).isEqualTo(TEST_REFRESH_TOKEN);
                        assertThat(r.data().getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.data().getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.data().getRole()).isEqualTo(Role.OPERATOR.name());
                    });

            then(authService).should().login(any(LoginRequest.class));
        }

        @Test
        @DisplayName("존재하지 않는 사용자로 로그인 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFound() {
            // given
            given(authService.login(any(LoginRequest.class)))
                    .willThrow(new ApiException(ErrorCode.USER_NOT_FOUND));

            // when & then
            assertThatThrownBy(() -> authController.login(loginRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(authService).should().login(any(LoginRequest.class));
        }

        @Test
        @DisplayName("잘못된 비밀번호로 로그인 시 예외가 발생한다")
        void shouldThrowExceptionWhenInvalidCredentials() {
            // given
            given(authService.login(any(LoginRequest.class)))
                    .willThrow(new ApiException(ErrorCode.INVALID_CREDENTIALS));

            // when & then
            assertThatThrownBy(() -> authController.login(loginRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.INVALID_CREDENTIALS);
                    });

            then(authService).should().login(any(LoginRequest.class));
        }
    }

    @Nested
    @DisplayName("로그아웃 테스트")
    class LogoutTest {

        @Test
        @DisplayName("유효한 리프레시 토큰으로 로그아웃이 성공한다")
        void shouldLogoutSuccessfullyWithValidRefreshToken() {
            // given
            willDoNothing().given(authService).logout(TEST_REFRESH_TOKEN);

            // when
            ApiResponse<Void> response = authController.logout(logoutRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("로그아웃이 완료되었습니다");
                        assertThat(r.data()).isNull();
                    });

            then(authService).should().logout(TEST_REFRESH_TOKEN);
        }

        @Test
        @DisplayName("무효한 리프레시 토큰으로 로그아웃 시 예외가 발생한다")
        void shouldThrowExceptionWhenInvalidRefreshToken() {
            // given
            willThrow(new ApiException(ErrorCode.TOKEN_INVALID))
                    .given(authService).logout(TEST_REFRESH_TOKEN);

            // when & then
            assertThatThrownBy(() -> authController.logout(logoutRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.TOKEN_INVALID);
                    });

            then(authService).should().logout(TEST_REFRESH_TOKEN);
        }
    }

    @Nested
    @DisplayName("토큰 갱신 테스트")
    class RefreshTokenTest {

        @Test
        @DisplayName("유효한 리프레시 토큰으로 토큰 갱신이 성공한다")
        void shouldRefreshTokenSuccessfullyWithValidRefreshToken() {
            // given
            LoginResponse refreshResponse = LoginResponse.builder()
                    .accessToken("new.access.token")
                    .refreshToken("new.refresh.token")
                    .userId(TEST_USER_ID)
                    .username(TEST_USERNAME)
                    .role(Role.OPERATOR.name())
                    .build();

            given(authService.refreshToken(TEST_REFRESH_TOKEN))
                    .willReturn(refreshResponse);

            // when
            ApiResponse<LoginResponse> response = authController.refreshToken(logoutRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("토큰 갱신이 완료되었습니다");
                        assertThat(r.data()).isNotNull();
                        assertThat(r.data().getAccessToken()).isEqualTo("new.access.token");
                        assertThat(r.data().getRefreshToken()).isEqualTo("new.refresh.token");
                        assertThat(r.data().getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.data().getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.data().getRole()).isEqualTo(Role.OPERATOR.name());
                    });

            then(authService).should().refreshToken(TEST_REFRESH_TOKEN);
        }

        @Test
        @DisplayName("무효한 리프레시 토큰으로 토큰 갱신 시 예외가 발생한다")
        void shouldThrowExceptionWhenInvalidRefreshTokenForRefresh() {
            // given
            given(authService.refreshToken(TEST_REFRESH_TOKEN))
                    .willThrow(new ApiException(ErrorCode.TOKEN_INVALID));

            // when & then
            assertThatThrownBy(() -> authController.refreshToken(logoutRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.TOKEN_INVALID);
                    });

            then(authService).should().refreshToken(TEST_REFRESH_TOKEN);
        }

        @Test
        @DisplayName("존재하지 않는 사용자의 토큰 갱신 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFoundForRefresh() {
            // given
            given(authService.refreshToken(TEST_REFRESH_TOKEN))
                    .willThrow(new ApiException(ErrorCode.USER_NOT_FOUND));

            // when & then
            assertThatThrownBy(() -> authController.refreshToken(logoutRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(authService).should().refreshToken(TEST_REFRESH_TOKEN);
        }
    }

    @Nested
    @DisplayName("비즈니스 로직 호출 검증 테스트")
    class BusinessLogicInvocationTest {

        @Test
        @DisplayName("로그인 시 올바른 파라미터로 서비스가 호출된다")
        void shouldCallLoginServiceWithCorrectParameters() {
            // given
            given(authService.login(loginRequest)).willReturn(loginResponse);

            // when
            authController.login(loginRequest);

            // then
            then(authService).should().login(loginRequest);
        }

        @Test
        @DisplayName("로그아웃 시 올바른 토큰으로 서비스가 호출된다")
        void shouldCallLogoutServiceWithCorrectToken() {
            // given
            willDoNothing().given(authService).logout(TEST_REFRESH_TOKEN);

            // when
            authController.logout(logoutRequest);

            // then
            then(authService).should().logout(TEST_REFRESH_TOKEN);
        }

        @Test
        @DisplayName("토큰 갱신 시 올바른 토큰으로 서비스가 호출된다")
        void shouldCallRefreshTokenServiceWithCorrectToken() {
            // given
            given(authService.refreshToken(TEST_REFRESH_TOKEN)).willReturn(loginResponse);

            // when
            authController.refreshToken(logoutRequest);

            // then
            then(authService).should().refreshToken(TEST_REFRESH_TOKEN);
        }
    }
}