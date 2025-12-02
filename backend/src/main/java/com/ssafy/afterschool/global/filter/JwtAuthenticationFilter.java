package com.ssafy.afterschool.global.filter;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.afterschool.domain.auth.service.TokenBlacklistServiceImpl;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.security.JwtProvider;
import com.ssafy.afterschool.domain.auth.service.TokenBlacklistService;
import com.ssafy.afterschool.global.security.UserPrincipal;
import io.jsonwebtoken.ExpiredJwtException;
import io.jsonwebtoken.MalformedJwtException;
import io.jsonwebtoken.UnsupportedJwtException;
import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import java.io.IOException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.stereotype.Component;
import org.springframework.util.StringUtils;
import org.springframework.web.filter.OncePerRequestFilter;

/**
 * JWT 토큰 검증 및 인증 처리 필터
 */
@Slf4j
@Component
@RequiredArgsConstructor
public class JwtAuthenticationFilter extends OncePerRequestFilter {

    private final JwtProvider jwtProvider;
    private final TokenBlacklistService tokenBlacklistService;
    private final ObjectMapper objectMapper;

    @Override
    protected void doFilterInternal(HttpServletRequest request, HttpServletResponse response,
                                    FilterChain filterChain) throws ServletException, IOException {
        try {
            String token = extractTokenFromRequest(request);

            if (StringUtils.hasText(token)) {
                // 1. JWT 기본 검증
                if (jwtProvider.validateToken(token)) {
                    
                    // 2. 블랙리스트 체크
                    if (tokenBlacklistService.isBlacklisted(token)) {
                        log.warn("Blacklisted token attempted to access");
                        handleJwtException(response, ErrorCode.TOKEN_INVALID);
                        return;
                    }

                    // 3. 사용자 정보 추출
                    Long userId = jwtProvider.getUserIdFromToken(token);
                    String username = jwtProvider.getUsernameFromToken(token);
                    
                    // 4. 사용자별 강제 로그아웃 체크 (TokenBlacklistServiceImpl 캐스팅 필요)
                    if (tokenBlacklistService instanceof TokenBlacklistServiceImpl blacklistServiceImpl) {
                        if (blacklistServiceImpl.isUserTokenBlacklisted(userId, jwtProvider.getIssuedAtFromToken(token))) {
                            log.warn("User {} token blacklisted by admin", username);
                            handleJwtException(response, ErrorCode.TOKEN_INVALID);
                            return;
                        }
                    }

                    // 5. UserPrincipal 생성 및 SecurityContext 설정
                    UserPrincipal userPrincipal = new UserPrincipal(
                            userId,
                            username,
                            jwtProvider.getRoleFromToken(token).name()
                    );

                    UsernamePasswordAuthenticationToken authentication =
                            new UsernamePasswordAuthenticationToken(
                                    userPrincipal,
                                    null,
                                    userPrincipal.getAuthorities()
                            );

                    SecurityContextHolder.getContext().setAuthentication(authentication);
                    log.debug("User authenticated: {} with role: {}", userPrincipal.getUsername(), userPrincipal.getRole());
                }
            }

        } catch (ExpiredJwtException e) {
            log.warn("Token has expired: {}", e.getMessage());
            handleJwtException(response, ErrorCode.TOKEN_EXPIRED);
            return;

        } catch (UnsupportedJwtException | MalformedJwtException | SecurityException e) {
            log.warn("JWT token is invalid: {}", e.getMessage());
            handleJwtException(response, ErrorCode.TOKEN_INVALID);
            return;

        } catch (IllegalArgumentException e) {
            log.warn("JWT token is malformed: {}", e.getMessage());
            handleJwtException(response, ErrorCode.TOKEN_INVALID);
            return;

        } catch (Exception e) {
            log.error("Unexpected error during JWT authentication: ", e);
            handleJwtException(response, ErrorCode.INTERNAL_SERVER_ERROR);
            return;
        }

        filterChain.doFilter(request, response);
    }

    /**
     * Request에서 JWT 토큰 추출
     */
    private String extractTokenFromRequest(HttpServletRequest request) {
        String bearerToken = request.getHeader("Authorization");

        if (StringUtils.hasText(bearerToken) && bearerToken.startsWith("Bearer ")) {
            return bearerToken.substring(7);
        }

        return null;
    }

    /**
     * JWT 예외 발생 시 공통 에러 형식으로 응답 전송
     */
    private void handleJwtException(HttpServletResponse response, ErrorCode  errorCode)
            throws IOException {

        response.setStatus(errorCode.getStatus().value());
        response.setContentType("application/json;charset=UTF-8");

        ApiResponse<Void> errorResponse = ApiResponse.fail(errorCode);

        String jsonResponse = objectMapper.writeValueAsString(errorResponse);
        response.getWriter().write(jsonResponse);
    }

    /**
     * 특정 경로는 JWT 검증 제외
     */
    @Override
    protected boolean shouldNotFilter(HttpServletRequest request)
            throws ServletException {
        String path = request.getRequestURI();

        // 인증이 필요 없는 경로들
        return path.startsWith("/api/auth/") ||
                path.startsWith("/api/public/") ||
                path.equals("/api/health") ||
                path.startsWith("/actuator/") ||
                path.equals("/") ||
                path.startsWith("/static/") ||
                path.startsWith("/css/") ||
                path.startsWith("/js/") ||
                path.startsWith("/images/");
    }
}