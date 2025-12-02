package com.ssafy.afterschool.domain.auth.service;

import com.ssafy.afterschool.domain.auth.dto.LoginRequest;
import com.ssafy.afterschool.domain.auth.dto.LoginResponse;
import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.domain.user.repository.UserRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.security.JwtProvider;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

/**
 * 인증 서비스 구현체
 */
@Slf4j
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class AuthServiceImpl implements AuthService {

    private final UserRepository userRepository;
    private final PasswordEncoder passwordEncoder;
    private final JwtProvider jwtProvider;
    private final TokenBlacklistService tokenBlacklistService;

    /**
     * 로그인
     * JWT 토큰을 생성하여 반환하면, 이후 요청에서는 JwtAuthenticationFilter가
     * 토큰을 검증하고 UserPrincipal을 생성하여 SecurityContext에 저장
     */
    @Override
    @Transactional
    public LoginResponse login(LoginRequest request) {
        // 1. 사용자 조회
        User user = userRepository.findByUsernameAndIsDeletedFalse(request.getUsername())
                .orElseThrow(() -> new ApiException(ErrorCode.USER_NOT_FOUND));

        // 2. 비밀번호 검증
        if (!passwordEncoder.matches(request.getPassword(), user.getPassword())) {
            throw new ApiException(ErrorCode.INVALID_CREDENTIALS);
        }

        // 3. 마지막 로그인 시간 업데이트
        user.updateLastLoginAt();

        // 4. JWT 토큰 생성 (이후 요청에서 UserPrincipal로 변환됨)
        String accessToken = jwtProvider.generateAccessToken(user.getId(), user.getUsername(), user.getRole());
        String refreshToken = jwtProvider.generateRefreshToken(user.getId(), user.getUsername(), user.getRole());

        log.info("User logged in successfully: {}", user.getUsername());

        // 5. 응답 생성
        return LoginResponse.builder()
                .accessToken(accessToken)
                .refreshToken(refreshToken)
                .userId(user.getId())
                .username(user.getUsername())
                .role(user.getRole().name())
                .build();
    }

    /**
     * 로그아웃
     * 리프레시 토큰을 블랙리스트에 추가하여 무효화
     */
    @Override
    public void logout(String refreshToken) {
        // JWT 토큰 유효성 검증
        if (!jwtProvider.validateToken(refreshToken)) {
            throw new ApiException(ErrorCode.TOKEN_INVALID);
        }

        // 토큰 정보 추출
        String username = jwtProvider.getUsernameFromToken(refreshToken);
        
        // 토큰을 블랙리스트에 추가 (남은 만료 시간만큼 TTL 설정)
        long remainingTimeSeconds = jwtProvider.getRemainingExpirationTimeInSeconds(refreshToken);
        if (remainingTimeSeconds > 0) {
            tokenBlacklistService.addToBlacklist(refreshToken, remainingTimeSeconds);
        }

        log.info("User logged out and token blacklisted: {}", username);
    }

    /**
     * 토큰 갱신
     * 기존 리프레시 토큰을 블랙리스트에 추가하여 재사용 방지
     */
    @Override
    public LoginResponse refreshToken(String refreshToken) {
        // 1. 리프레시 토큰 검증
        if (!jwtProvider.validateToken(refreshToken)) {
            throw new ApiException(ErrorCode.TOKEN_INVALID);
        }

        // 2. 토큰에서 사용자 정보 추출
        Long userId = jwtProvider.getUserIdFromToken(refreshToken);
        String username = jwtProvider.getUsernameFromToken(refreshToken);
        
        // 3. 사용자 존재 여부 확인
        User user = userRepository.findByIdAndIsDeletedFalse(userId)
                .orElseThrow(() -> new ApiException(ErrorCode.USER_NOT_FOUND));

        // 4. 기존 리프레시 토큰을 블랙리스트에 추가 (재사용 방지)
        long remainingTimeSeconds = jwtProvider.getRemainingExpirationTimeInSeconds(refreshToken);
        if (remainingTimeSeconds > 0) {
            tokenBlacklistService.addToBlacklist(refreshToken, remainingTimeSeconds);
        }

        // 5. 새 토큰 생성
        String newAccessToken = jwtProvider.generateAccessToken(user.getId(), user.getUsername(), user.getRole());
        String newRefreshToken = jwtProvider.generateRefreshToken(user.getId(), user.getUsername(), user.getRole());

        log.debug("Token refreshed for user: {} (old token blacklisted)", username);

        // 6. 응답 생성
        return LoginResponse.builder()
                .accessToken(newAccessToken)
                .refreshToken(newRefreshToken)
                .userId(user.getId())
                .username(user.getUsername())
                .role(user.getRole().name())
                .build();
    }
}