package com.ssafy.afterschool.global.filter;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.afterschool.domain.auth.service.TokenBlacklistService;
import com.ssafy.afterschool.domain.auth.service.TokenBlacklistServiceImpl;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Role;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.security.JwtProvider;
import com.ssafy.afterschool.global.security.UserPrincipal;
import io.jsonwebtoken.ExpiredJwtException;
import io.jsonwebtoken.MalformedJwtException;
import io.jsonwebtoken.UnsupportedJwtException;
import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.security.core.context.SecurityContext;
import org.springframework.security.core.context.SecurityContextHolder;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Date;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("JwtAuthenticationFilter 단위 테스트")
class JwtAuthenticationFilterTest {

    @Mock
    private JwtProvider jwtProvider;
    
    @Mock
    private TokenBlacklistService tokenBlacklistService;
    
    @Mock
    private ObjectMapper objectMapper;
    
    @Mock
    private HttpServletRequest request;
    
    @Mock
    private HttpServletResponse response;
    
    @Mock
    private FilterChain filterChain;
    
    @Mock
    private SecurityContext securityContext;

    @InjectMocks
    private JwtAuthenticationFilter jwtAuthenticationFilter;

    private static final String VALID_TOKEN = "Bearer valid.jwt.token";
    private static final String JWT_TOKEN = "valid.jwt.token";
    private static final Long TEST_USER_ID = 1L;
    private static final String TEST_USERNAME = "testuser";
    private static final String TEST_ROLE = Role.OPERATOR.name();

    @BeforeEach
    void setUp() {
        SecurityContextHolder.setContext(securityContext);
    }

    @Nested
    @DisplayName("토큰 추출 테스트")
    class TokenExtractionTest {

        @Test
        @DisplayName("유효한 Authorization 헤더에서 토큰을 추출한다")
        void shouldExtractTokenFromValidAuthorizationHeader() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willReturn(true);
            given(tokenBlacklistService.isBlacklisted(JWT_TOKEN)).willReturn(false);
            given(jwtProvider.getUserIdFromToken(JWT_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(JWT_TOKEN)).willReturn(TEST_USERNAME);
            given(jwtProvider.getRoleFromToken(JWT_TOKEN)).willReturn(Role.OPERATOR);

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(jwtProvider).should().validateToken(JWT_TOKEN);
            then(filterChain).should().doFilter(request, response);
        }

        @Test
        @DisplayName("Authorization 헤더가 없으면 토큰 추출하지 않는다")
        void shouldNotExtractTokenWhenNoAuthorizationHeader() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(null);

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(jwtProvider).shouldHaveNoInteractions();
            then(filterChain).should().doFilter(request, response);
        }

        @ParameterizedTest
        @ValueSource(strings = {"", " ", "Invalid token", "Basic dGVzdA==", "Bearer"})
        @DisplayName("잘못된 형식의 Authorization 헤더는 무시한다")
        void shouldIgnoreInvalidAuthorizationHeader(String invalidHeader) throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(invalidHeader);

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(jwtProvider).shouldHaveNoInteractions();
            then(filterChain).should().doFilter(request, response);
        }
    }

    @Nested
    @DisplayName("토큰 검증 및 인증 테스트")
    class TokenValidationAndAuthenticationTest {

        @Test
        @DisplayName("유효한 토큰으로 인증이 성공한다")
        void shouldAuthenticateWithValidToken() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willReturn(true);
            given(tokenBlacklistService.isBlacklisted(JWT_TOKEN)).willReturn(false);
            given(jwtProvider.getUserIdFromToken(JWT_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(JWT_TOKEN)).willReturn(TEST_USERNAME);
            given(jwtProvider.getRoleFromToken(JWT_TOKEN)).willReturn(Role.OPERATOR);

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(securityContext).should().setAuthentication(any());
            then(filterChain).should().doFilter(request, response);
        }

        @Test
        @DisplayName("블랙리스트된 토큰은 인증을 거부한다")
        void shouldRejectBlacklistedToken() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willReturn(true);
            given(tokenBlacklistService.isBlacklisted(JWT_TOKEN)).willReturn(true);
            
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            given(response.getWriter()).willReturn(printWriter);
            given(objectMapper.writeValueAsString(any(ApiResponse.class))).willReturn("{}");

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(response).should().setStatus(ErrorCode.TOKEN_INVALID.getStatus().value());
            then(response).should().setContentType("application/json;charset=UTF-8");
            then(filterChain).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("사용자별 강제 로그아웃된 토큰은 인증을 거부한다")
        void shouldRejectUserBlacklistedToken() throws ServletException, IOException {
            // given
            TokenBlacklistServiceImpl blacklistServiceImpl = mock(TokenBlacklistServiceImpl.class);
            JwtAuthenticationFilter filter = new JwtAuthenticationFilter(jwtProvider, blacklistServiceImpl, objectMapper);
            
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willReturn(true);
            given(blacklistServiceImpl.isBlacklisted(JWT_TOKEN)).willReturn(false);
            given(jwtProvider.getUserIdFromToken(JWT_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(JWT_TOKEN)).willReturn(TEST_USERNAME);
            given(jwtProvider.getIssuedAtFromToken(JWT_TOKEN)).willReturn(new Date());
            given(blacklistServiceImpl.isUserTokenBlacklisted(eq(TEST_USER_ID), any(Date.class))).willReturn(true);
            
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            given(response.getWriter()).willReturn(printWriter);
            given(objectMapper.writeValueAsString(any(ApiResponse.class))).willReturn("{}");

            // when
            filter.doFilterInternal(request, response, filterChain);

            // then
            then(response).should().setStatus(ErrorCode.TOKEN_INVALID.getStatus().value());
            then(filterChain).shouldHaveNoInteractions();
        }
    }

    @Nested
    @DisplayName("JWT 예외 처리 테스트")
    class JwtExceptionHandlingTest {

        @Test
        @DisplayName("만료된 토큰에 대해 적절한 에러를 반환한다")
        void shouldHandleExpiredJwtException() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willThrow(new ExpiredJwtException(null, null, "Token expired"));
            
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            given(response.getWriter()).willReturn(printWriter);
            given(objectMapper.writeValueAsString(any(ApiResponse.class))).willReturn("{}");

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(response).should().setStatus(ErrorCode.TOKEN_EXPIRED.getStatus().value());
            then(filterChain).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("잘못된 형식의 토큰에 대해 적절한 에러를 반환한다")
        void shouldHandleMalformedJwtException() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willThrow(new MalformedJwtException("Malformed token"));
            
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            given(response.getWriter()).willReturn(printWriter);
            given(objectMapper.writeValueAsString(any(ApiResponse.class))).willReturn("{}");

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(response).should().setStatus(ErrorCode.TOKEN_INVALID.getStatus().value());
            then(filterChain).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("지원되지 않는 토큰에 대해 적절한 에러를 반환한다")
        void shouldHandleUnsupportedJwtException() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willThrow(new UnsupportedJwtException("Unsupported token"));
            
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            given(response.getWriter()).willReturn(printWriter);
            given(objectMapper.writeValueAsString(any(ApiResponse.class))).willReturn("{}");

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(response).should().setStatus(ErrorCode.TOKEN_INVALID.getStatus().value());
            then(filterChain).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("예상치 못한 예외에 대해 내부 서버 에러를 반환한다")
        void shouldHandleUnexpectedException() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willThrow(new RuntimeException("Unexpected error"));
            
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            given(response.getWriter()).willReturn(printWriter);
            given(objectMapper.writeValueAsString(any(ApiResponse.class))).willReturn("{}");

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(response).should().setStatus(ErrorCode.INTERNAL_SERVER_ERROR.getStatus().value());
            then(filterChain).shouldHaveNoInteractions();
        }
    }

    @Nested
    @DisplayName("필터 제외 경로 테스트")
    class FilterExclusionTest {

        @ParameterizedTest
        @ValueSource(strings = {
            "/api/auth/login",
            "/api/auth/logout", 
            "/api/public/status",
            "/api/health",
            "/actuator/health",
            "/",
            "/static/css/style.css",
            "/css/main.css",
            "/js/app.js",
            "/images/logo.png"
        })
        @DisplayName("인증이 필요 없는 경로는 필터를 거치지 않는다")
        void shouldSkipFilterForExcludedPaths(String path) throws ServletException {
            // given
            given(request.getRequestURI()).willReturn(path);

            // when
            boolean shouldNotFilter = jwtAuthenticationFilter.shouldNotFilter(request);

            // then
            assertThat(shouldNotFilter)
                    .isTrue()
                    .describedAs("경로 '%s'는 JWT 검증을 거치지 않아야 함", path);
        }

        @ParameterizedTest
        @ValueSource(strings = {
            "/api/users/profile",
            "/api/admin/users",
            "/api/robots/status",
            "/api/maps/list"
        })
        @DisplayName("인증이 필요한 경로는 필터를 거친다")
        void shouldApplyFilterForProtectedPaths(String path) throws ServletException {
            // given
            given(request.getRequestURI()).willReturn(path);

            // when
            boolean shouldNotFilter = jwtAuthenticationFilter.shouldNotFilter(request);

            // then
            assertThat(shouldNotFilter)
                    .isFalse()
                    .describedAs("경로 '%s'는 JWT 검증을 거쳐야 함", path);
        }
    }

    @Nested
    @DisplayName("UserPrincipal 생성 테스트")  
    class UserPrincipalCreationTest {

        @Test
        @DisplayName("유효한 토큰 정보로 UserPrincipal을 생성한다")
        void shouldCreateUserPrincipalWithValidTokenInfo() throws ServletException, IOException {
            // given
            given(request.getHeader("Authorization")).willReturn(VALID_TOKEN);
            given(jwtProvider.validateToken(JWT_TOKEN)).willReturn(true);
            given(tokenBlacklistService.isBlacklisted(JWT_TOKEN)).willReturn(false);
            given(jwtProvider.getUserIdFromToken(JWT_TOKEN)).willReturn(TEST_USER_ID);
            given(jwtProvider.getUsernameFromToken(JWT_TOKEN)).willReturn(TEST_USERNAME);
            given(jwtProvider.getRoleFromToken(JWT_TOKEN)).willReturn(Role.OPERATOR);

            // when
            jwtAuthenticationFilter.doFilterInternal(request, response, filterChain);

            // then
            then(securityContext).should().setAuthentication(argThat(auth -> {
                UserPrincipal principal = (UserPrincipal) auth.getPrincipal();
                return principal.getId().equals(TEST_USER_ID) &&
                       principal.getUsername().equals(TEST_USERNAME) &&
                       principal.getRole().equals(TEST_ROLE);
            }));
        }
    }
}