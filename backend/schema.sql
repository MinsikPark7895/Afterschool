-- SSAFY 순찰 로봇 시뮬레이션 데이터베이스 스키마
-- 프로젝트: 교내 순찰 로봇 시뮬레이션 (ROS2 + Spring Boot + AWS S3)
-- 생성일: 2025-09-04

-- 문자셋 설정
SET NAMES utf8mb4;

-- 데이터베이스 생성
CREATE DATABASE IF NOT EXISTS AfterSchool 
CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
USE AfterSchool;

-- =====================================================
-- 1. 사용자 테이블
-- =====================================================
CREATE TABLE users (
                       id BIGINT AUTO_INCREMENT PRIMARY KEY,
                       username VARCHAR(50) UNIQUE NOT NULL COMMENT 'admin, operator1',
                       password VARCHAR(255) NOT NULL COMMENT 'BCrypt 암호화',
                       name VARCHAR(100) NOT NULL COMMENT '관리자 실명',
                       role VARCHAR(20) DEFAULT 'OPERATOR' COMMENT 'ADMIN, OPERATOR',
                       last_login_at TIMESTAMP NULL,
                       is_deleted BOOLEAN DEFAULT FALSE,
                       created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                       updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
) COMMENT '시스템 사용자 관리';

-- 최초 관리자 계정 (개발/배포 시 생성)
-- 비밀번호는 환경 변수에서 설정하거나 초기 설정 시 변경 필요
INSERT INTO users (username, password, name, role)
VALUES ('admin', '$2a$10$9TR7tttIb9Qg4QloygKeU.MeXCi8V2Krz9gSpeO2kKcWxDmdqskNC', '관리자', 'ADMIN');

-- =====================================================
-- 2. 로봇 기본 정보 테이블
-- =====================================================
CREATE TABLE robots (
                        id BIGINT AUTO_INCREMENT PRIMARY KEY,
                        robot_id VARCHAR(50) UNIQUE NOT NULL COMMENT 'tb1, tb2 (비즈니스 키)',
                        robot_name VARCHAR(100) NOT NULL COMMENT '순찰로봇 A',
                        is_active BOOLEAN DEFAULT TRUE,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
) COMMENT '로봇 기본 정보 및 설정';

-- =====================================================
-- 3. 이벤트 로그 테이블 (MQTT 이벤트)
-- =====================================================
CREATE TABLE events (
                        id BIGINT AUTO_INCREMENT PRIMARY KEY,
                        event_type VARCHAR(50) NOT NULL COMMENT 'detection, mission_done, map_complete',
                        robot_id VARCHAR(50) COMMENT 'robots.robot_id 참조 (NULL 가능)',
                        severity VARCHAR(20) DEFAULT 'info' COMMENT 'critical, warning, info',
                        location_data JSON COMMENT 'ROS 좌표 + 픽셀 좌표 + 구역 정보',
                        detection_data JSON COMMENT '탐지 관련 상세 데이터 (confidence, bounding_box 등)',
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

                        CONSTRAINT fk_events_robot_id FOREIGN KEY (robot_id) REFERENCES robots(robot_id) ON DELETE SET NULL
) COMMENT '침입자 탐지 및 시스템 이벤트';

-- =====================================================
-- 4. 증거 파일 테이블 (S3 키 저장용)
-- =====================================================
CREATE TABLE evidence_files (
                                id BIGINT AUTO_INCREMENT PRIMARY KEY,
                                event_id BIGINT NOT NULL,
                                s3_bucket VARCHAR(100) NOT NULL COMMENT 'S3 버킷명',
                                s3_key VARCHAR(500) NOT NULL COMMENT 'S3 객체 키',
                                original_filename VARCHAR(255) NOT NULL,
                                file_size BIGINT NOT NULL,
                                mime_type VARCHAR(100) NOT NULL COMMENT 'image/jpeg, video/mp4',
                                metadata JSON COMMENT '해상도, 촬영시간 등',
                                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

                                CONSTRAINT fk_evidence_files_event_id FOREIGN KEY (event_id) REFERENCES events(id) ON DELETE CASCADE
) COMMENT '침입자 탐지 증거 파일 (S3 저장)';

-- =====================================================
-- 5. MQTT 명령 로그 테이블
-- =====================================================
CREATE TABLE command_log (
                             id BIGINT AUTO_INCREMENT PRIMARY KEY,
                             target_robot_id VARCHAR(50) COMMENT 'robots.robot_id 참조 (NULL이면 시스템 명령)',
                             user_id BIGINT NOT NULL COMMENT 'users.id 참조 - 명령 실행자',
                             command_type VARCHAR(50) NOT NULL COMMENT 'start_patrol, stop_patrol, move_to',
                             command_payload JSON NOT NULL COMMENT 'MQTT to_robot payload',
                             command_status VARCHAR(20) COMMENT 'sent, success, failed',
                             response_data JSON COMMENT '로봇 응답 데이터',
                             sent_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                             completed_at TIMESTAMP NULL,

                             CONSTRAINT fk_command_log_user_id FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
                             CONSTRAINT fk_command_log_target_robot_id FOREIGN KEY (target_robot_id) REFERENCES robots(robot_id) ON DELETE SET NULL
) COMMENT 'Spring Boot에서 ROS2로 보낸 명령 추적';
