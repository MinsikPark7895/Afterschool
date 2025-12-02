package com.ssafy.afterschool.domain.auth.service;

import com.ssafy.afterschool.global.security.JwtProvider;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.util.Date;

/**
 * JWT 토큰 블랙리스트 서비스 구현체
 * Redis를 사용하여 블랙리스트 토큰 관리
 */
@Slf4j
@Service
@RequiredArgsConstructor
public class TokenBlacklistServiceImpl implements TokenBlacklistService {

    private final RedisTemplate<String, String> redisTemplate;
    private final JwtProvider jwtProvider;

    private static final String BLACKLIST_KEY_PREFIX = "blacklist:jti:";
    private static final String USER_BLACKLIST_KEY_PREFIX = "blacklist:user:";

    /**
     * 토큰을 블랙리스트에 추가
     * JWT ID(jti)를 키로 사용하여 Redis에 저장하고, 토큰 만료 시간과 동일하게 TTL 설정
     */
    @Override
    public void addToBlacklist(String token, long expirationSeconds) {
        try {
            String jti = jwtProvider.getJtiFromToken(token);
            String key = BLACKLIST_KEY_PREFIX + jti;
            redisTemplate.opsForValue().set(key, "blacklisted", Duration.ofSeconds(expirationSeconds));
            log.info("Token JTI added to blacklist: {} with expiration: {} seconds", jti, expirationSeconds);
        } catch (Exception e) {
            log.warn("Failed to add token to blacklist, Redis may be unavailable: {}", e.getMessage());
            // Redis가 사용 불가능한 경우에도 로그아웃 자체는 성공으로 처리
            // 토큰은 만료 시간이 되면 자동으로 무효화됨
        }
    }

    /**
     * 토큰이 블랙리스트에 있는지 확인
     * JWT ID(jti)를 사용하여 블랙리스트 여부 확인
     */
    @Override
    public boolean isBlacklisted(String token) {
        try {
            String jti = jwtProvider.getJtiFromToken(token);
            String key = BLACKLIST_KEY_PREFIX + jti;
            Boolean exists = redisTemplate.hasKey(key);
            return exists != null && exists;
        } catch (Exception e) {
            log.error("Failed to check token blacklist status: ", e);
            // Redis 연결 실패 시 보수적으로 접근을 허용하지 않음
            return true;
        }
    }

    /**
     * 사용자의 모든 토큰을 블랙리스트에 추가 (강제 로그아웃)
     * 사용자별 블랙리스트 타임스탬프를 설정하여, 해당 시간 이전에 발급된 모든 토큰을 무효화
     */
    @Override
    public void blacklistAllUserTokens(Long userId) {
        try {
            String key = USER_BLACKLIST_KEY_PREFIX + userId;
            long currentTime = System.currentTimeMillis();
            
            // 사용자의 블랙리스트 타임스탬프를 현재 시간으로 설정
            // 이 시간 이전에 발급된 모든 토큰은 무효가 됨
            redisTemplate.opsForValue().set(key, String.valueOf(currentTime), Duration.ofDays(30));
            
            log.info("All tokens blacklisted for user: {}", userId);
        } catch (Exception e) {
            log.warn("Failed to blacklist all user tokens for user {}, Redis may be unavailable: {}", userId, e.getMessage());
            // Redis가 사용 불가능한 경우에도 강제 로그아웃 자체는 성공으로 처리
            // 새로운 토큰 발급을 차단하는 다른 메커니즘이 필요할 수 있음
        }
    }

    /**
     * 사용자 토큰이 강제 로그아웃되었는지 확인
     * @param userId 사용자 ID
     * @param tokenIssuedAt 토큰 발급 시간
     * @return 강제 로그아웃되었으면 true
     */
    public boolean isUserTokenBlacklisted(Long userId, Date tokenIssuedAt) {
        try {
            String key = USER_BLACKLIST_KEY_PREFIX + userId;
            String blacklistTimeStr = redisTemplate.opsForValue().get(key);
            
            if (blacklistTimeStr == null) {
                return false;
            }
            
            long blacklistTime = Long.parseLong(blacklistTimeStr);
            return tokenIssuedAt.getTime() < blacklistTime;
            
        } catch (Exception e) {
            log.error("Failed to check user token blacklist status for user {}: ", userId, e);
            // Redis 연결 실패 시 보수적으로 접근을 허용하지 않음
            return true;
        }
    }
}