package com.ssafy.afterschool.global.security;

import com.ssafy.afterschool.global.constants.Role;
import io.jsonwebtoken.JwtException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.junit.jupiter.MockitoExtension;

import java.time.Instant;
import java.util.Date;

import static org.assertj.core.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("JwtProvider 단위 테스트")
class JwtProviderTest {

    private JwtProvider jwtProvider;
    
    private static final String TEST_SECRET = "testSecretKeyForJWTWhichShouldBeLongEnoughForHS256Algorithm";
    private static final long ACCESS_TOKEN_EXPIRATION = 3600000L; // 1시간
    private static final long REFRESH_TOKEN_EXPIRATION = 604800000L; // 1주일
    
    private static final Long TEST_USER_ID = 1L;
    private static final String TEST_USERNAME = "testuser";
    private static final Role TEST_ROLE = Role.OPERATOR;

    @BeforeEach
    void setUp() {
        jwtProvider = new JwtProvider(TEST_SECRET, ACCESS_TOKEN_EXPIRATION, REFRESH_TOKEN_EXPIRATION);
    }

    @Nested
    @DisplayName("토큰 생성 테스트")
    class TokenGenerationTest {

        @Test
        @DisplayName("Access Token이 올바른 형식으로 생성된다")
        void shouldGenerateValidAccessToken() {
            // when
            String token = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);

            // then
            assertThat(token)
                    .isNotNull()
                    .isNotEmpty()
                    .contains(".");
            
            assertThat(token.split("\\."))
                    .hasSize(3)
                    .describedAs("JWT는 header.payload.signature 3부분으로 구성되어야 함");
        }

        @Test
        @DisplayName("Refresh Token이 올바른 형식으로 생성된다")
        void shouldGenerateValidRefreshToken() {
            // when
            String token = jwtProvider.generateRefreshToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);

            // then
            assertThat(token)
                    .isNotNull()
                    .isNotEmpty()
                    .contains(".");
            
            assertThat(token.split("\\."))
                    .hasSize(3);
        }

        @ParameterizedTest
        @EnumSource(Role.class)
        @DisplayName("모든 역할에 대해 토큰이 생성된다")
        void shouldGenerateTokenForAllRoles(Role role) {
            // when
            String token = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, role);

            // then
            assertThat(token).isNotNull();
            assertThat(jwtProvider.getRoleFromToken(token)).isEqualTo(role);
        }

        @Test
        @DisplayName("각 토큰마다 고유한 JWT ID가 생성된다")
        void shouldGenerateUniqueJtiForEachToken() {
            // when
            String token1 = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);
            String token2 = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);

            // then
            String jti1 = jwtProvider.getJtiFromToken(token1);
            String jti2 = jwtProvider.getJtiFromToken(token2);
            
            assertThat(jti1)
                    .isNotEqualTo(jti2)
                    .describedAs("각 토큰은 고유한 JTI를 가져야 함");
        }
    }

    @Nested
    @DisplayName("토큰 검증 테스트")
    class TokenValidationTest {

        @Test
        @DisplayName("유효한 토큰 검증이 성공한다")
        void shouldValidateValidToken() {
            // given
            String token = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);

            // when & then
            assertThatCode(() -> jwtProvider.validateToken(token))
                    .doesNotThrowAnyException();
        }

        @ParameterizedTest
        @ValueSource(strings = {
            "invalid.jwt.token",
            "eyJhbGciOiJIUzI1NiJ9.invalid.signature",
            "not-a-jwt-at-all"
        })
        @DisplayName("잘못된 토큰 검증이 실패한다")
        void shouldFailToValidateInvalidToken(String invalidToken) {
            // when & then
            assertThatThrownBy(() -> jwtProvider.validateToken(invalidToken))
                    .isInstanceOf(JwtException.class)
                    .describedAs("잘못된 토큰은 JwtException을 발생시켜야 함");
        }

        @Test
        @DisplayName("빈 토큰 검증이 실패한다")
        void shouldFailToValidateEmptyToken() {
            // when & then
            assertThatThrownBy(() -> jwtProvider.validateToken(""))
                    .isInstanceOf(IllegalArgumentException.class)
                    .hasMessageContaining("cannot be null or empty");
        }

        @Test
        @DisplayName("만료되지 않은 토큰이 유효하다고 확인된다")
        void shouldIdentifyNonExpiredTokenAsValid() {
            // given
            String token = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);

            // when
            boolean isExpired = jwtProvider.isTokenExpired(token);

            // then
            assertThat(isExpired)
                    .isFalse()
                    .describedAs("새로 생성된 토큰은 만료되지 않아야 함");
        }
    }

    @Nested
    @DisplayName("클레임 추출 테스트")
    class ClaimExtractionTest {

        private String validToken;

        @BeforeEach
        void setUpToken() {
            validToken = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);
        }

        @Test
        @DisplayName("토큰에서 사용자 ID를 올바르게 추출한다")
        void shouldExtractUserIdFromToken() {
            // when
            Long extractedUserId = jwtProvider.getUserIdFromToken(validToken);

            // then
            assertThat(extractedUserId)
                    .isEqualTo(TEST_USER_ID)
                    .describedAs("추출된 사용자 ID는 원본과 일치해야 함");
        }

        @Test
        @DisplayName("토큰에서 사용자명을 올바르게 추출한다")
        void shouldExtractUsernameFromToken() {
            // when
            String extractedUsername = jwtProvider.getUsernameFromToken(validToken);

            // then
            assertThat(extractedUsername)
                    .isEqualTo(TEST_USERNAME)
                    .describedAs("추출된 사용자명은 원본과 일치해야 함");
        }

        @Test
        @DisplayName("토큰에서 역할을 올바르게 추출한다")
        void shouldExtractRoleFromToken() {
            // when
            Role extractedRole = jwtProvider.getRoleFromToken(validToken);

            // then
            assertThat(extractedRole)
                    .isEqualTo(TEST_ROLE)
                    .describedAs("추출된 역할은 원본과 일치해야 함");
        }

        @Test
        @DisplayName("토큰에서 JWT ID를 올바르게 추출한다")
        void shouldExtractJtiFromToken() {
            // when
            String jti = jwtProvider.getJtiFromToken(validToken);

            // then
            assertThat(jti)
                    .isNotNull()
                    .isNotEmpty()
                    .matches("[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")
                    .describedAs("JTI는 UUID 형식이어야 함");
        }

        @Test
        @DisplayName("토큰에서 발급 시간을 올바르게 추출한다")
        void shouldExtractIssuedAtFromToken() {
            // given
            Instant beforeGeneration = Instant.now().truncatedTo(java.time.temporal.ChronoUnit.SECONDS);
            String token = jwtProvider.generateAccessToken(TEST_USER_ID, TEST_USERNAME, TEST_ROLE);
            Instant afterGeneration = Instant.now().truncatedTo(java.time.temporal.ChronoUnit.SECONDS).plusSeconds(1);

            // when
            Date issuedAt = jwtProvider.getIssuedAtFromToken(token);

            // then
            assertThat(issuedAt.toInstant())
                    .isBetween(beforeGeneration, afterGeneration)
                    .describedAs("발급 시간은 토큰 생성 시점과 일치해야 함");
        }

        @Test
        @DisplayName("토큰의 남은 만료 시간을 올바르게 계산한다")
        void shouldCalculateRemainingExpirationTime() {
            // when
            long remainingTime = jwtProvider.getRemainingExpirationTimeInSeconds(validToken);

            // then
            assertThat(remainingTime)
                    .isPositive()
                    .isLessThanOrEqualTo(ACCESS_TOKEN_EXPIRATION / 1000)
                    .describedAs("남은 만료 시간은 양수이고 최대 만료 시간보다 작아야 함");
        }
    }

    @Nested
    @DisplayName("예외 처리 테스트")
    class ExceptionHandlingTest {

        @ParameterizedTest
        @ValueSource(strings = {"invalid.jwt.token", "not-a-jwt"})
        @DisplayName("잘못된 토큰에서 클레임 추출 시 JwtException이 발생한다")
        void shouldThrowJwtExceptionWhenExtractingClaimsFromInvalidToken(String invalidToken) {
            // when & then
            assertThatThrownBy(() -> jwtProvider.getUserIdFromToken(invalidToken))
                    .isInstanceOf(JwtException.class);
            
            assertThatThrownBy(() -> jwtProvider.getUsernameFromToken(invalidToken))
                    .isInstanceOf(JwtException.class);
            
            assertThatThrownBy(() -> jwtProvider.getRoleFromToken(invalidToken))
                    .isInstanceOf(JwtException.class);
            
            assertThatThrownBy(() -> jwtProvider.getJtiFromToken(invalidToken))
                    .isInstanceOf(JwtException.class);
        }

        @Test
        @DisplayName("빈 토큰에서 클레임 추출 시 IllegalArgumentException이 발생한다")
        void shouldThrowIllegalArgumentExceptionWhenExtractingClaimsFromEmptyToken() {
            // when & then
            assertThatThrownBy(() -> jwtProvider.getUserIdFromToken(""))
                    .isInstanceOf(IllegalArgumentException.class)
                    .hasMessageContaining("cannot be null or empty");
            
            assertThatThrownBy(() -> jwtProvider.getUsernameFromToken(""))
                    .isInstanceOf(IllegalArgumentException.class)
                    .hasMessageContaining("cannot be null or empty");
            
            assertThatThrownBy(() -> jwtProvider.getRoleFromToken(""))
                    .isInstanceOf(IllegalArgumentException.class)
                    .hasMessageContaining("cannot be null or empty");
            
            assertThatThrownBy(() -> jwtProvider.getJtiFromToken(""))
                    .isInstanceOf(IllegalArgumentException.class)
                    .hasMessageContaining("cannot be null or empty");
        }

        @Test
        @DisplayName("잘못된 토큰의 만료 시간 확인 시 true를 반환한다")
        void shouldReturnTrueForExpiredTimeCheckOnInvalidToken() {
            // given
            String invalidToken = "invalid.jwt.token";

            // when
            boolean isExpired = jwtProvider.isTokenExpired(invalidToken);

            // then
            assertThat(isExpired)
                    .isTrue()
                    .describedAs("잘못된 토큰은 만료된 것으로 간주해야 함");
        }

        @Test
        @DisplayName("잘못된 토큰의 남은 만료 시간 확인 시 0을 반환한다")
        void shouldReturnZeroForRemainingTimeOnInvalidToken() {
            // given
            String invalidToken = "invalid.jwt.token";

            // when
            long remainingTime = jwtProvider.getRemainingExpirationTimeInSeconds(invalidToken);

            // then
            assertThat(remainingTime)
                    .isZero()
                    .describedAs("잘못된 토큰의 남은 시간은 0이어야 함");
        }
    }
}