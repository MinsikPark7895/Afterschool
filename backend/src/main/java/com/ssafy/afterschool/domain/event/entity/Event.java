package com.ssafy.afterschool.domain.event.entity;

import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import org.hibernate.annotations.JdbcTypeCode;
import org.hibernate.type.SqlTypes;

import java.time.LocalDateTime;

/**
 * 이벤트 로그 엔티티 (MQTT 이벤트)
 * - 침입자 탐지, 임무 완료, 맵 완성 등 모든 이벤트 저장
 * - WebSocket 알림 및 이력 관리용
 */
@Entity
@Table(name = "events")
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class Event {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Enumerated(EnumType.STRING)
    @Column(name = "event_type", nullable = false, length = 50)
    private EventType eventType;

    @Column(name = "robot_id", length = 50)
    private String robotId;

    @Enumerated(EnumType.STRING)
    @Column(name = "severity", length = 20)
    private Severity severity;

    @JdbcTypeCode(SqlTypes.JSON)
    @Column(name = "location_data", columnDefinition = "JSON")
    private String locationData; // ROS 좌표 + 픽셀 좌표 + 구역 정보

    @JdbcTypeCode(SqlTypes.JSON)
    @Column(name = "detection_data", columnDefinition = "JSON")
    private String detectionData; // 탐지 관련 상세 데이터 (confidence, bounding_box 등)

    @CreationTimestamp
    @Column(name = "created_at", nullable = false)
    private LocalDateTime createdAt;
}
