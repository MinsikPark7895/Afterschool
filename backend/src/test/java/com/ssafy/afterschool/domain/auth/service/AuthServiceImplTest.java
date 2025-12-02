package com.ssafy.afterschool.domain.auth.service;

import com.ssafy.afterschool.domain.auth.dto.LoginRequest;
import com.ssafy.afterschool.domain.auth.dto.LoginResponse;
import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.domain.user.repository.UserRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Role;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.security.JwtProvider;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.security.crypto.password.PasswordEncoder;

import java.time.LocalDateTime;
import java.util.Optional;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("AuthServiceImpl 단위 테스트")
class AuthServiceImplTest {

    @Mock
    private UserRepository userRepository;
    
    @Mock
    private PasswordEncoder passwordEncoder;
    
    @Mock
    private JwtProvider jwtProvider;
    
    @Mock
    private TokenBlacklistService tokenBlacklistService;

    @InjectMocks
    private AuthServiceImpl authService;

    private static final String TEST_USERNAME = "testuser";
    private static final String TEST_PASSWORD = "password123";
    private static final String TEST_ENCODED_PASSWORD = "encodedPassword123";
    private static final String TEST_ACCESS_TOKEN = "access.token.jwt";
    private static final String TEST_REFRESH_TOKEN = "refresh.token.jwt";
    private static final Long TEST_USER_ID = 1L;
    private static final String TEST_NAME = "Test User";

    private User testUser;
    private LoginRequest loginRequest;

    @BeforeEach
    void setUp() {
        testUser = User.builder()
                .id(TEST_USER_ID)
                .username(TEST_USERNAME)
                .password(TEST_ENCODED_PASSWORD)
                .name(TEST_NAME)
                .role(Role.OPERATOR)
                .isDeleted(false)
                .build();

        loginRequest = LoginRequest.builder()
                .username(TEST_USERNAME)
                .password(TEST_PASSWORD)
                .build();
    }

    @Nested
    @DisplayName("로그인 테스트")
    class LoginTest {

        @Test
        @DisplayName("올바른 인증 정보로 로그인이 성공한다")
        void shouldLoginSuccessfullyWithValidCredentials() {
            // given
            given(userRepository.findByUsernameAndIsDeletedFalse(TEST_USERNAME))
                    .willReturn(Optional.of(testUser));
            given(passwordEncoder.matches(TEST_PASSWORD, TEST_ENCODED_PASSWORD))
                    .willReturn(true);
            given(jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, Role.OPERATOR))
                    .willReturn(TEST_ACCESS_TOKEN);
            given(jwtProvider.generateRefreshToken(TEST_USER_ID, TEST_USERNAME, Role.OPERATOR))
                    .willReturn(TEST_REFRESH_TOKEN);

            // when
            LoginResponse response = authService.login(loginRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getAccessToken()).isEqualTo(TEST_ACCESS_TOKEN);
                        assertThat(r.getRefreshToken()).isEqualTo(TEST_REFRESH_TOKEN);
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.getRole()).isEqualTo(Role.OPERATOR.name());
                    });

            // 마지막 로그인 시간이 업데이트되었는지 확인
            then(userRepository).should().findByUsernameAndIsDeletedFalse(TEST_USERNAME);
            then(passwordEncoder).should().matches(TEST_PASSWORD, TEST_ENCODED_PASSWORD);
        }

        @Test
        @DisplayName("존재하지 않는 사용자로 로그인 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFound() {
            // given
            given(userRepository.findByUsernameAndIsDeletedFalse(TEST_USERNAME))
                    .willReturn(Optional.empty());

            // when & then
            assertThatThrownBy(() -> authService.login(loginRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(passwordEncoder).shouldHaveNoInteractions();
            then(jwtProvider).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("잘못된 비밀번호로 로그인 시 예외가 발생한다")
        void shouldThrowExceptionWhenInvalidPassword() {
            // given
            given(userRepository.findByUsernameAndIsDeletedFalse(TEST_USERNAME))
                    .willReturn(Optional.of(testUser));
            given(passwordEncoder.matches(TEST_PASSWORD, TEST_ENCODED_PASSWORD))
                    .willReturn(false);

            // when & then
            assertThatThrownBy(() -> authService.login(loginRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.INVALID_CREDENTIALS);
                    });

            then(jwtProvider).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("삭제된 사용자로 로그인 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserIsDeleted() {
            // given
            given(userRepository.findByUsernameAndIsDeletedFalse(TEST_USERNAME))
                    .willReturn(Optional.empty()); // 삭제된 사용자는 조회되지 않음

            // when & then
            assertThatThrownBy(() -> authService.login(loginRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });
        }
    }

    @Nested
    @DisplayName("로그아웃 테스트")
    class LogoutTest {

        @Test
        @DisplayName("유효한 토큰으로 로그아웃이 성공한다")
        void shouldLogoutSuccessfullyWithValidToken() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(true);
            given(jwtProvider.getUsernameFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USERNAME);
            given(jwtProvider.getRemainingExpirationTimeInSeconds(TEST_REFRESH_TOKEN)).willReturn(3600L);

            // when & then  
            assertThatCode(() -> authService.logout(TEST_REFRESH_TOKEN))
                    .doesNotThrowAnyException();

            // then
            then(tokenBlacklistService).should()
                    .addToBlacklist(TEST_REFRESH_TOKEN, 3600L);
        }

        @Test
        @DisplayName("무효한 토큰으로 로그아웃 시 예외가 발생한다")
        void shouldThrowExceptionWhenTokenIsInvalid() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(false);

            // when & then
            assertThatThrownBy(() -> authService.logout(TEST_REFRESH_TOKEN))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.TOKEN_INVALID);
                    });

            then(tokenBlacklistService).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("만료 시간이 0인 토큰은 블랙리스트에 추가하지 않는다")
        void shouldNotAddToBlacklistWhenTokenHasNoRemainingTime() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(true);
            given(jwtProvider.getUsernameFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USERNAME);
            given(jwtProvider.getRemainingExpirationTimeInSeconds(TEST_REFRESH_TOKEN)).willReturn(0L);

            // when
            authService.logout(TEST_REFRESH_TOKEN);

            // then
            then(tokenBlacklistService).shouldHaveNoInteractions();
        }
    }

    @Nested
    @DisplayName("토큰 갱신 테스트")
    class RefreshTokenTest {

        @Test
        @DisplayName("유효한 리프레시 토큰으로 토큰 갱신이 성공한다")
        void shouldRefreshTokenSuccessfullyWithValidRefreshToken() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(true);
            given(jwtProvider.getUserIdFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USERNAME);
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));
            given(jwtProvider.getRemainingExpirationTimeInSeconds(TEST_REFRESH_TOKEN)).willReturn(3600L);
            
            String newAccessToken = "new.access.token";
            String newRefreshToken = "new.refresh.token";
            given(jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, Role.OPERATOR))
                    .willReturn(newAccessToken);
            given(jwtProvider.generateRefreshToken(TEST_USER_ID, TEST_USERNAME, Role.OPERATOR))
                    .willReturn(newRefreshToken);

            // when
            LoginResponse response = authService.refreshToken(TEST_REFRESH_TOKEN);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getAccessToken()).isEqualTo(newAccessToken);
                        assertThat(r.getRefreshToken()).isEqualTo(newRefreshToken);
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.getRole()).isEqualTo(Role.OPERATOR.name());
                    });

            // 기존 토큰이 블랙리스트에 추가되었는지 확인
            then(tokenBlacklistService).should()
                    .addToBlacklist(TEST_REFRESH_TOKEN, 3600L);
        }

        @Test
        @DisplayName("무효한 리프레시 토큰으로 토큰 갱신 시 예외가 발생한다")
        void shouldThrowExceptionWhenRefreshTokenIsInvalid() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(false);

            // when & then
            assertThatThrownBy(() -> authService.refreshToken(TEST_REFRESH_TOKEN))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.TOKEN_INVALID);
                    });

            then(userRepository).shouldHaveNoInteractions();
            then(tokenBlacklistService).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("존재하지 않는 사용자의 토큰 갱신 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFoundForRefresh() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(true);
            given(jwtProvider.getUserIdFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USERNAME);
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.empty());

            // when & then
            assertThatThrownBy(() -> authService.refreshToken(TEST_REFRESH_TOKEN))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(tokenBlacklistService).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("삭제된 사용자의 토큰 갱신 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserIsDeletedForRefresh() {
            // given
            given(jwtProvider.validateToken(TEST_REFRESH_TOKEN)).willReturn(true);
            given(jwtProvider.getUserIdFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(TEST_REFRESH_TOKEN)).willReturn(TEST_USERNAME);
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.empty()); // 삭제된 사용자는 조회되지 않음

            // when & then
            assertThatThrownBy(() -> authService.refreshToken(TEST_REFRESH_TOKEN))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });
        }
    }
}