package com.ssafy.afterschool.domain.user.repository;

import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.global.constants.Role;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

/**
 * User 엔티티 리포지토리 - JWT + UserPrincipal 방식
 */
@Repository
public interface UserRepository extends JpaRepository<User, Long> {

    // ================================
    // 1. JWT 기반 인증용 메서드
    // ================================

    /**
     * ID로 활성 사용자 조회 (JWT 토큰에서 ID 추출 후 사용)
     * 가장 많이 사용되는 핵심 메서드
     */
    Optional<User> findByIdAndIsDeletedFalse(Long id);

    /**
     * 로그인 시 username으로 사용자 조회 (최초 인증용)
     */
    Optional<User> findByUsernameAndIsDeletedFalse(String username);

    // ================================
    // 2. 사용자 관리용 메서드
    // ================================

    /**
     * 사용자명 중복 확인 (사용자 생성 시)
     */
    boolean existsByUsernameAndIsDeletedFalse(String username);

    // ================================
    // 3. 관리자 페이지용 조회 메서드
    // ================================

    /**
     * 모든 활성 사용자 조회 (페이징)
     */
    Page<User> findByIsDeletedFalse(Pageable pageable);

    /**
     * 권한별 활성 사용자 조회 (페이징)
     */
    Page<User> findByRoleAndIsDeletedFalse(Role role, Pageable pageable);

    // ================================
    // 4. 비즈니스 로직 지원 메서드
    // ================================

    /**
     * 특정 권한 활성 사용자 수 조회
     */
    long countByRoleAndIsDeletedFalse(Role role);
}