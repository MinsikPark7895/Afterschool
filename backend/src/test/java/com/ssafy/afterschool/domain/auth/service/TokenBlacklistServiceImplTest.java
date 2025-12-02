package com.ssafy.afterschool.domain.auth.service;

import com.ssafy.afterschool.global.security.JwtProvider;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.data.redis.core.ValueOperations;

import java.time.Duration;
import java.util.Date;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("TokenBlacklistServiceImpl 단위 테스트")
class TokenBlacklistServiceImplTest {

    @Mock
    private RedisTemplate<String, String> redisTemplate;
    
    @Mock
    private JwtProvider jwtProvider;
    
    @Mock
    private ValueOperations<String, String> valueOperations;

    @InjectMocks
    private TokenBlacklistServiceImpl tokenBlacklistService;

    private static final String TEST_TOKEN = "test.jwt.token";
    private static final String TEST_JTI = "550e8400-e29b-41d4-a716-446655440000";
    private static final Long TEST_USER_ID = 1L;
    private static final long TEST_EXPIRATION_SECONDS = 3600L;

    @BeforeEach
    void setUp() {
        lenient().when(redisTemplate.opsForValue()).thenReturn(valueOperations);
    }

    @Nested
    @DisplayName("토큰 블랙리스트 추가 테스트")
    class AddToBlacklistTest {

        @Test
        @DisplayName("토큰을 성공적으로 블랙리스트에 추가한다")
        void shouldAddTokenToBlacklistSuccessfully() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN)).willReturn(TEST_JTI);

            // when & then
            assertThatCode(() -> tokenBlacklistService.addToBlacklist(TEST_TOKEN, TEST_EXPIRATION_SECONDS))
                    .doesNotThrowAnyException();

            // then
            String expectedKey = "blacklist:jti:" + TEST_JTI;
            then(valueOperations).should()
                    .set(expectedKey, "blacklisted", Duration.ofSeconds(TEST_EXPIRATION_SECONDS));
        }

        @Test
        @DisplayName("JTI 추출 실패 시 graceful하게 처리한다")
        void shouldHandleJtiExtractionFailureGracefully() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN))
                    .willThrow(new RuntimeException("Invalid token"));

            // when & then (예외가 던져지지 않아야 함)
            assertThatCode(() -> tokenBlacklistService.addToBlacklist(TEST_TOKEN, TEST_EXPIRATION_SECONDS))
                    .doesNotThrowAnyException();
        }

        @Test
        @DisplayName("Redis 작업 실패 시 graceful하게 처리한다")
        void shouldHandleRedisOperationFailureGracefully() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN)).willReturn(TEST_JTI);
            willThrow(new RuntimeException("Redis connection failed"))
                    .given(valueOperations).set(anyString(), anyString(), any(Duration.class));

            // when & then (예외가 던져지지 않아야 함)
            assertThatCode(() -> tokenBlacklistService.addToBlacklist(TEST_TOKEN, TEST_EXPIRATION_SECONDS))
                    .doesNotThrowAnyException();
        }
    }

    @Nested
    @DisplayName("토큰 블랙리스트 확인 테스트")
    class IsBlacklistedTest {

        @Test
        @DisplayName("블랙리스트에 있는 토큰을 올바르게 식별한다")
        void shouldIdentifyBlacklistedToken() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN)).willReturn(TEST_JTI);
            given(redisTemplate.hasKey("blacklist:jti:" + TEST_JTI)).willReturn(true);

            // when
            boolean isBlacklisted = tokenBlacklistService.isBlacklisted(TEST_TOKEN);

            // then
            assertThat(isBlacklisted)
                    .isTrue()
                    .describedAs("블랙리스트에 있는 토큰은 true를 반환해야 함");
        }

        @Test
        @DisplayName("블랙리스트에 없는 토큰을 올바르게 식별한다")
        void shouldIdentifyNonBlacklistedToken() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN)).willReturn(TEST_JTI);
            given(redisTemplate.hasKey("blacklist:jti:" + TEST_JTI)).willReturn(false);

            // when
            boolean isBlacklisted = tokenBlacklistService.isBlacklisted(TEST_TOKEN);

            // then
            assertThat(isBlacklisted)
                    .isFalse()
                    .describedAs("블랙리스트에 없는 토큰은 false를 반환해야 함");
        }

        @Test
        @DisplayName("Redis에서 null 반환 시 블랙리스트에 없는 것으로 처리한다")
        void shouldHandleNullResponseFromRedis() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN)).willReturn(TEST_JTI);
            given(redisTemplate.hasKey("blacklist:jti:" + TEST_JTI)).willReturn(null);

            // when
            boolean isBlacklisted = tokenBlacklistService.isBlacklisted(TEST_TOKEN);

            // then
            assertThat(isBlacklisted)
                    .isFalse()
                    .describedAs("Redis에서 null 반환 시 블랙리스트에 없는 것으로 처리해야 함");
        }

        @Test
        @DisplayName("Redis 연결 실패 시 보수적으로 블랙리스트된 것으로 처리한다")
        void shouldReturnTrueWhenRedisConnectionFails() {
            // given
            given(jwtProvider.getJtiFromToken(TEST_TOKEN)).willReturn(TEST_JTI);
            given(redisTemplate.hasKey(anyString()))
                    .willThrow(new RuntimeException("Redis connection failed"));

            // when
            boolean isBlacklisted = tokenBlacklistService.isBlacklisted(TEST_TOKEN);

            // then
            assertThat(isBlacklisted)
                    .isTrue()
                    .describedAs("Redis 연결 실패 시 보수적으로 블랙리스트된 것으로 처리해야 함");
        }
    }

    @Nested
    @DisplayName("사용자 전체 토큰 블랙리스트 테스트")
    class BlacklistAllUserTokensTest {

        @Test
        @DisplayName("사용자의 모든 토큰을 성공적으로 블랙리스트에 추가한다")
        void shouldBlacklistAllUserTokensSuccessfully() {
            // when & then
            assertThatCode(() -> tokenBlacklistService.blacklistAllUserTokens(TEST_USER_ID))
                    .doesNotThrowAnyException();

            // then
            String expectedKey = "blacklist:user:" + TEST_USER_ID;
            then(valueOperations).should()
                    .set(eq(expectedKey), anyString(), eq(Duration.ofDays(30)));
        }

        @Test
        @DisplayName("Redis 작업 실패 시 graceful하게 처리한다")
        void shouldHandleRedisOperationFailureGracefully() {
            // given
            willThrow(new RuntimeException("Redis connection failed"))
                    .given(valueOperations).set(anyString(), anyString(), any(Duration.class));

            // when & then (예외가 던져지지 않아야 함)
            assertThatCode(() -> tokenBlacklistService.blacklistAllUserTokens(TEST_USER_ID))
                    .doesNotThrowAnyException();
        }
    }

    @Nested
    @DisplayName("사용자 토큰 블랙리스트 확인 테스트")
    class IsUserTokenBlacklistedTest {

        private Date tokenIssuedAt;
        private Date beforeBlacklistTime;
        private Date afterBlacklistTime;

        @BeforeEach
        void setUpDates() {
            long currentTime = System.currentTimeMillis();
            beforeBlacklistTime = new Date(currentTime - 10000); // 10초 전
            tokenIssuedAt = new Date(currentTime - 5000); // 5초 전
            afterBlacklistTime = new Date(currentTime + 5000); // 5초 후
        }

        @Test
        @DisplayName("블랙리스트 시간 이전에 발급된 토큰을 올바르게 식별한다")
        void shouldIdentifyTokenIssuedBeforeBlacklistTime() {
            // given
            String blacklistTimeStr = String.valueOf(afterBlacklistTime.getTime());
            given(valueOperations.get("blacklist:user:" + TEST_USER_ID))
                    .willReturn(blacklistTimeStr);

            // when
            boolean isBlacklisted = tokenBlacklistService.isUserTokenBlacklisted(TEST_USER_ID, tokenIssuedAt);

            // then
            assertThat(isBlacklisted)
                    .isTrue()
                    .describedAs("블랙리스트 시간 이전에 발급된 토큰은 블랙리스트된 것으로 처리해야 함");
        }

        @Test
        @DisplayName("블랙리스트 시간 이후에 발급된 토큰을 올바르게 식별한다")
        void shouldIdentifyTokenIssuedAfterBlacklistTime() {
            // given
            String blacklistTimeStr = String.valueOf(beforeBlacklistTime.getTime());
            given(valueOperations.get("blacklist:user:" + TEST_USER_ID))
                    .willReturn(blacklistTimeStr);

            // when
            boolean isBlacklisted = tokenBlacklistService.isUserTokenBlacklisted(TEST_USER_ID, tokenIssuedAt);

            // then
            assertThat(isBlacklisted)
                    .isFalse()
                    .describedAs("블랙리스트 시간 이후에 발급된 토큰은 유효한 것으로 처리해야 함");
        }

        @Test
        @DisplayName("사용자 블랙리스트가 없는 경우 유효한 것으로 처리한다")
        void shouldReturnFalseWhenNoUserBlacklistExists() {
            // given
            given(valueOperations.get("blacklist:user:" + TEST_USER_ID)).willReturn(null);

            // when
            boolean isBlacklisted = tokenBlacklistService.isUserTokenBlacklisted(TEST_USER_ID, tokenIssuedAt);

            // then
            assertThat(isBlacklisted)
                    .isFalse()
                    .describedAs("사용자 블랙리스트가 없으면 토큰은 유효한 것으로 처리해야 함");
        }

        @Test
        @DisplayName("Redis 연결 실패 시 보수적으로 블랙리스트된 것으로 처리한다")
        void shouldReturnTrueWhenRedisConnectionFails() {
            // given
            given(valueOperations.get(anyString()))
                    .willThrow(new RuntimeException("Redis connection failed"));

            // when
            boolean isBlacklisted = tokenBlacklistService.isUserTokenBlacklisted(TEST_USER_ID, tokenIssuedAt);

            // then
            assertThat(isBlacklisted)
                    .isTrue()
                    .describedAs("Redis 연결 실패 시 보수적으로 블랙리스트된 것으로 처리해야 함");
        }

        @Test
        @DisplayName("잘못된 블랙리스트 시간 형식 시 예외로 처리한다")
        void shouldHandleInvalidBlacklistTimeFormat() {
            // given
            given(valueOperations.get("blacklist:user:" + TEST_USER_ID))
                    .willReturn("invalid-time-format");

            // when
            boolean isBlacklisted = tokenBlacklistService.isUserTokenBlacklisted(TEST_USER_ID, tokenIssuedAt);

            // then
            assertThat(isBlacklisted)
                    .isTrue()
                    .describedAs("잘못된 시간 형식은 예외 상황으로 블랙리스트된 것으로 처리해야 함");
        }
    }
}