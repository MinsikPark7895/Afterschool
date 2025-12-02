package com.ssafy.afterschool.domain.robot.service;

import com.ssafy.afterschool.domain.robot.dto.*;
import com.ssafy.afterschool.domain.robot.entity.Robot;
import com.ssafy.afterschool.global.mqtt.MqttService;
import com.ssafy.afterschool.global.mqtt.CommandType;
import com.ssafy.afterschool.domain.robot.repository.RobotRepository;
import com.ssafy.afterschool.global.aspect.LogRobotCommand;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.coordinate.dto.Position;
import com.ssafy.afterschool.global.coordinate.service.CoordinateTransformService;
import com.ssafy.afterschool.global.exception.ApiException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.stream.Collectors;

/**
 * 로봇 서비스 구현체
 * - 로봇 관리 비즈니스 로직
 * - 기존 좌표 변환 시스템 활용
 * - 책임 분리: MQTT 메시지 생성/전송은 별도 컴포넌트
 */
@Slf4j
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class RobotServiceImpl implements RobotService {

    private final RobotRepository robotRepository;
    private final CoordinateTransformService coordinateTransformService;
    private final MqttService mqttService;

    @Override
    public List<RobotInfoResponse> getAllRobots() {
        log.info("📋 모든 로봇 목록 조회");
        try {
            return robotRepository.findAll()
                    .stream()
                    .map(RobotInfoResponse::from)
                    .toList();
        } catch (Exception e) {
            log.error("❌ 로봇 목록 조회 실패", e);
            throw new ApiException(ErrorCode.DATABASE_OPERATION_FAILED);
        }
    }

    @Override
    public RobotInfoResponse getRobotById(String robotId) {
        log.info("🤖 로봇 조회 - robotId: {}", robotId);
        Robot robot = robotRepository.findByRobotId(robotId)
                .orElseThrow(() -> new ApiException(ErrorCode.ROBOT_NOT_FOUND));
        
        return RobotInfoResponse.from(robot);
    }

    @Override
    @Transactional
    @LogRobotCommand(commandType = CommandType.MOVE_TO)
    public CommandResponse moveRobot(String robotId, MoveToRequest request) {
        log.info("🎮 로봇 이동 명령 - robotId: {}, target: ({}, {})", 
                robotId, request.getTargetPosition().getX(), request.getTargetPosition().getY());

        // 1. 로봇 존재 및 상태 확인
        validateRobotForCommand(robotId);

        // 2. 픽셀 좌표 → ROS 좌표 변환 (기존 시스템 활용)
        Position rosPosition = coordinateTransformService.pixelToRos(request.getTargetPosition());
        log.debug("🔄 좌표 변환 완료 - 픽셀({}, {}) → ROS({}, {})",
                request.getTargetPosition().getX(), request.getTargetPosition().getY(),
                rosPosition.getX(), rosPosition.getY());

        // 3. 픽셀 좌표 경계 검증
        if (!coordinateTransformService.isPixelInBounds(request.getTargetPosition())) {
            log.warn("⚠️ 목표 위치가 맵 경계를 벗어남 - robotId: {}", robotId);
            throw new ApiException(ErrorCode.INVALID_REQUEST_PARAMETER);
        }

        // 4. MQTT 명령 전송
        return mqttService.sendMoveCommand(robotId, rosPosition, request);
    }

    @Override
    @Transactional
    @LogRobotCommand(commandType = CommandType.START_PATROL)
    public CommandResponse startPatrol() {
        log.info("🚶 순찰 시작 명령 (일괄)");

        // 1. 활성화된 로봇 목록 조회
        List<String> activeRobots = getActiveRobotIds();
        if (activeRobots.isEmpty()) {
            log.warn("⚠️ 활성화된 로봇이 없음");
            throw new ApiException(ErrorCode.ROBOT_NOT_AVAILABLE);
        }

        // 2. MQTT 순찰 시작 명령 전송
        return mqttService.sendStartPatrolCommand(activeRobots);
    }

    @Override
    @Transactional
    @LogRobotCommand(commandType = CommandType.STOP_PATROL)
    public CommandResponse stopPatrol() {
        log.info("🛑 순찰 중지 명령 (일괄)");

        // 1. 활성화된 로봇 목록 조회
        List<String> activeRobots = getActiveRobotIds();
        if (activeRobots.isEmpty()) {
            log.warn("⚠️ 활성화된 로봇이 없음");
            throw new ApiException(ErrorCode.ROBOT_NOT_AVAILABLE);
        }

        // 2. MQTT 순찰 중지 명령 전송
        return mqttService.sendStopPatrolCommand(activeRobots);
    }

    // ==================== Private Helper Methods ====================

    /**
     * 명령 실행 전 로봇 상태 검증
     */
    private void validateRobotForCommand(String robotId) {
        Robot robot = robotRepository.findByRobotId(robotId)
                .orElseThrow(() -> new ApiException(ErrorCode.ROBOT_NOT_FOUND));

        if (!robot.isActive()) {
            throw new ApiException(ErrorCode.ROBOT_NOT_AVAILABLE);
        }

        log.debug("✅ 로봇 상태 검증 완료 - robotId: {}, isActive: {}",
                robotId, true);
    }

    /**
     * 활성화된 로봇 ID 목록 조회
     */
    private List<String> getActiveRobotIds() {
        try {
            List<String> activeRobots = robotRepository.findAll()
                    .stream()
                    .filter(Robot::isActive)
                    .map(Robot::getRobotId)
                    .collect(Collectors.toList());
            log.info("🎯 활성화된 로봇 목록 조회 - 개수: {}", activeRobots.size());
            return activeRobots;
        } catch (Exception e) {
            log.error("❌ 활성화된 로봇 목록 조회 실패", e);
            throw new ApiException(ErrorCode.DATABASE_OPERATION_FAILED);
        }
    }
}
