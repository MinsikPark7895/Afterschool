package com.ssafy.afterschool.domain.robot.controller;

import com.ssafy.afterschool.domain.robot.dto.*;
import com.ssafy.afterschool.domain.robot.service.RobotService;
import com.ssafy.afterschool.global.constants.SuccessCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.security.access.prepost.PreAuthorize;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 로봇 제어 및 관리 컨트롤러
 * - 로봇 상태 조회 및 제어 명령 API
 * - MQTT를 통한 ROS2 시스템과의 통신
 */
@Tag(name = "Robot", description = "로봇 제어 및 관리 API")
@Slf4j
@RestController
@RequestMapping("/robots")
@RequiredArgsConstructor
public class RobotController {

    private final RobotService robotService;

    /**
     * 모든 로봇 목록 조회
     */
    @Operation(summary = "로봇 목록 조회", description = "시스템에 등록된 모든 로봇의 기본 정보를 조회합니다")
    @GetMapping
    public ResponseEntity<ApiResponse<List<RobotInfoResponse>>> getAllRobots() {
        log.info("📋 로봇 목록 조회 API 호출");
        List<RobotInfoResponse> robots = robotService.getAllRobots();
        return ResponseEntity.ok(ApiResponse.success(SuccessCode.ROBOT_LIST_SUCCESS, robots));
    }

    /**
     * 특정 로봇 조회
     */
    @Operation(summary = "특정 로봇 조회", description = "로봇 ID로 특정 로봇의 상세 정보를 조회합니다")
    @GetMapping("/{robotId}")
    public ResponseEntity<ApiResponse<RobotInfoResponse>> getRobotById(
            @Parameter(description = "로봇 ID", example = "tb1", required = true)
            @PathVariable String robotId) {
        log.info("🤖 로봇 조회 API 호출 - robotId: {}", robotId);
        RobotInfoResponse robot = robotService.getRobotById(robotId);
        return ResponseEntity.ok(ApiResponse.success(SuccessCode.ROBOT_DETAIL_SUCCESS, robot));
    }
    
    /**
     * 로봇 이동 명령
     */
    @Operation(summary = "로봇 이동", description = "로봇을 특정 위치로 이동시킵니다")
    @PostMapping("/move-to/{robotId}")
    public ResponseEntity<ApiResponse<CommandResponse>> moveRobot(
            @Parameter(description = "로봇 ID", example = "tb1", required = true)
            @PathVariable String robotId,
            @Valid @RequestBody MoveToRequest request) {
        log.info("🎮 로봇 이동 명령 API 호출 - robotId: {}", robotId);
        CommandResponse response = robotService.moveRobot(robotId, request);
        return ResponseEntity.ok(ApiResponse.success(SuccessCode.ROBOT_MOVE_SUCCESS, response));
    }

    /**
     * 순찰 시작 명령 (일괄)
     */
    @Operation(summary = "순찰 시작", description = "모든 활성화된 로봇의 순찰을 시작합니다")
    @PostMapping("/start-patrol")
    public ResponseEntity<ApiResponse<CommandResponse>> startPatrol() {
        log.info("🚶 순찰 시작 명령 API 호출");
        CommandResponse response = robotService.startPatrol();
        return ResponseEntity.ok(ApiResponse.success(SuccessCode.PATROL_START_SUCCESS, response));
    }

    /**
     * 순찰 중지 명령
     */
    @Operation(summary = "순찰 중지", description = "모든 활성화된 로봇의 순찰을 중지합니다")
    @PostMapping("/stop-patrol")
    public ResponseEntity<ApiResponse<CommandResponse>> stopPatrol() {
        log.info("🛑 순찰 중지 명령 API 호출");
        CommandResponse response = robotService.stopPatrol();
        return ResponseEntity.ok(ApiResponse.success(SuccessCode.PATROL_STOP_SUCCESS, response));
    }
}
