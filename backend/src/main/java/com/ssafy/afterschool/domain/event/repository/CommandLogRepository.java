package com.ssafy.afterschool.domain.event.repository;

import com.ssafy.afterschool.domain.event.entity.CommandLog;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.List;

/**
 * 명령 로그 Repository
 */
@Repository
public interface CommandLogRepository extends JpaRepository<CommandLog, Long> {

    /**
     * 사용자별 명령 이력 조회 (페이징)
     */
    Page<CommandLog> findByUserIdOrderBySentAtDesc(Long userId, Pageable pageable);

    /**
     * 로봇별 명령 이력 조회 (페이징)
     */
    Page<CommandLog> findByTargetRobotIdOrderBySentAtDesc(String targetRobotId, Pageable pageable);

    /**
     * 명령 타입별 조회 (페이징)
     */
    Page<CommandLog> findByCommandTypeOrderBySentAtDesc(String commandType, Pageable pageable);

}
