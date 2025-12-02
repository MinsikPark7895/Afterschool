package com.ssafy.afterschool.domain.robot.entity;

import com.ssafy.afterschool.global.constants.Status;
import jakarta.persistence.*;
import lombok.*;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import java.time.LocalDateTime;

/**
 * 로봇 엔티티
 * - 순찰 로봇의 기본 정보 관리
 * - ROS2 시뮬레이션 환경에서 동작하는 가상 로봇
 */
@Entity
@Table(name = "robots")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
@EntityListeners(AuditingEntityListener.class)
public class Robot {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "id")
    private Long id;

    /**
     * ROS2에서 사용하는 로봇 식별자
     * - "tb1", "tb2" 등
     * - MQTT 토픽에서 사용되는 식별자와 동일
     */
    @Column(name = "robot_id", unique = true, nullable = false, length = 50)
    private String robotId;

    /**
     * 사용자에게 표시되는 로봇 이름
     * - "순찰로봇 1호", "Robot A" 등
     */
    @Column(name = "robot_name", nullable = false, length = 100)
    private String robotName;

    /**
     * 로봇 활성화 상태
     * - true: 활성 상태 (순찰 가능)
     * - false: 비활성 상태 (점검 중, 오류 등)
     */
    @Column(name = "is_active", nullable = false)
    private boolean isActive;

    /**
     * 로봇 등록일시
     */
    @CreatedDate
    @Column(name = "created_at", nullable = false, updatable = false)
    private LocalDateTime createdAt;

    /**
     * 로봇 정보 수정일시
     */
    @LastModifiedDate
    @Column(name = "updated_at", nullable = false)
    private LocalDateTime updatedAt;
}
