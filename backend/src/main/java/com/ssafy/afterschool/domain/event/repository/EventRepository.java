package com.ssafy.afterschool.domain.event.repository;

import com.ssafy.afterschool.domain.event.entity.Event;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.List;

/**
 * 이벤트 Repository
 */
@Repository
public interface EventRepository extends JpaRepository<Event, Long> {

    /**
     * 침입자 탐지 이벤트 필터링 조회 (로봇ID, 심각도, 날짜 범위)
     */
    @Query("SELECT e FROM Event e WHERE e.eventType = 'DETECTION' AND " +
           "(:robotId IS NULL OR e.robotId = :robotId) AND " +
           "(:severity IS NULL OR e.severity = :severity) AND " +
           "(:startDate IS NULL OR e.createdAt >= :startDate) AND " +
           "(:endDate IS NULL OR e.createdAt <= :endDate) " +
           "ORDER BY e.createdAt DESC")
    Page<Event> findDetectionEventsByFilters(
            @Param("robotId") String robotId,
            @Param("severity") String severity,
            @Param("startDate") LocalDateTime startDate,
            @Param("endDate") LocalDateTime endDate,
            Pageable pageable
    );

}
