package com.ssafy.afterschool.domain.auth.service;

/**
 * JWT 토큰 블랙리스트 서비스 인터페이스
 */
public interface TokenBlacklistService {

    /**
     * 토큰을 블랙리스트에 추가
     * @param token JWT 토큰
     * @param expirationSeconds 토큰 만료 시간 (초)
     */
    void addToBlacklist(String token, long expirationSeconds);

    /**
     * 토큰이 블랙리스트에 있는지 확인
     * @param token JWT 토큰
     * @return 블랙리스트에 있으면 true, 없으면 false
     */
    boolean isBlacklisted(String token);

    /**
     * 사용자의 모든 토큰을 블랙리스트에 추가 (강제 로그아웃)
     * @param userId 사용자 ID
     */
    void blacklistAllUserTokens(Long userId);
}