package com.ssafy.afterschool.domain.event.entity;

import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import org.hibernate.annotations.JdbcTypeCode;
import org.hibernate.type.SqlTypes;

import java.time.LocalDateTime;

/**
 * MQTT 명령 로그 엔티티
 * - Spring Boot에서 ROS2로 보낸 명령 추적
 */
@Entity
@Table(name = "command_log")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class CommandLog {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "target_robot_id", length = 50)
    private String targetRobotId; // robots.robot_id 참조 (NULL이면 시스템 명령)

    @Column(name = "user_id", nullable = false)
    private Long userId; // users.id 참조 - 명령 실행자

    @Column(name = "command_type", nullable = false, length = 50)
    private String commandType; // start_patrol, stop_patrol, move_to

    @Column(name = "command_payload", nullable = false, columnDefinition = "JSON")
    private String commandPayload; // MQTT to_robot payload

    @Column(name = "command_status", length = 20)
    @Builder.Default
    private String commandStatus = "sent"; // sent, success, failed

    @Column(name = "response_data", columnDefinition = "JSON")
    private String responseData; // 로봇 응답 데이터

    @CreationTimestamp
    @Column(name = "sent_at", nullable = false)
    private LocalDateTime sentAt;

    @Column(name = "completed_at")
    private LocalDateTime completedAt;

    /**
     * 명령 완료 처리
     */
    public void markAsCompleted(String responseData) {
        this.commandStatus = "success";
        this.responseData = responseData;
        this.completedAt = LocalDateTime.now();
    }

    /**
     * 명령 실패 처리
     */
    public void markAsFailed(String errorResponse) {
        this.commandStatus = "failed";
        this.responseData = errorResponse;
        this.completedAt = LocalDateTime.now();
    }

    /**
     * 명령 상태 업데이트 (AOP용)
     */
    public void updateStatus(String status) {
        this.commandStatus = status;
        if ("success".equals(status) || "failed".equals(status)) {
            this.completedAt = LocalDateTime.now();
        }
    }
}
