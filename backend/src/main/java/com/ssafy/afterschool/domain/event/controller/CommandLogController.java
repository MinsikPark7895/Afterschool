package com.ssafy.afterschool.domain.event.controller;

import com.ssafy.afterschool.domain.event.dto.CommandLogResponse;
import com.ssafy.afterschool.domain.event.service.CommandLogService;
import com.ssafy.afterschool.global.constants.SuccessCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.response.PageResponse;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 로봇 명령 로그 컨트롤러
 * - 로봇 제어 명령 이력 조회
 * - 명령 실행 상태 추적
 */
@Tag(name = "Command Log", description = "로봇 명령 로그 관리 API")
@Slf4j
@RestController
@RequestMapping("/command-logs")
@RequiredArgsConstructor
public class CommandLogController {

    private final CommandLogService commandLogService;

    @Operation(summary = "로봇별 명령 로그 조회", description = "특정 로봇의 명령 실행 이력을 조회합니다.")
    @GetMapping("/robot/{robotId}")
    public ApiResponse<PageResponse<CommandLogResponse>> getCommandLogsByRobot(
            @Parameter(description = "로봇 ID") @PathVariable String robotId,
            @RequestParam(defaultValue = "0") int page,
            @RequestParam(defaultValue = "20") int size) {

        log.info("📋 로봇별 명령 로그 조회 - Robot: {}, Page: {}", robotId, page);

        Pageable pageable = PageRequest.of(page, size, Sort.by(Sort.Direction.DESC, "sentAt"));
        PageResponse<CommandLogResponse> logs = commandLogService.getCommandLogsByRobotId(robotId, pageable);
        
        return ApiResponse.success(SuccessCode.EVENT_LIST_SUCCESS, logs);
    }

    @Operation(summary = "명령 타입별 로그 조회", description = "특정 명령 타입의 실행 이력을 조회합니다.")
    @GetMapping("/type/{commandType}")
    public ApiResponse<PageResponse<CommandLogResponse>> getCommandLogsByType(
            @Parameter(description = "명령 타입") @PathVariable String commandType,
            @RequestParam(defaultValue = "0") int page,
            @RequestParam(defaultValue = "20") int size) {

        log.info("📋 명령 타입별 로그 조회 - Type: {}, Page: {}", commandType, page);

        Pageable pageable = PageRequest.of(page, size, Sort.by(Sort.Direction.DESC, "sentAt"));
        PageResponse<CommandLogResponse> logs = commandLogService.getCommandLogsByType(commandType, pageable);
        
        return ApiResponse.success(SuccessCode.EVENT_LIST_SUCCESS, logs);
    }


    @Operation(summary = "전체 명령 로그 조회", description = "모든 로봇의 명령 실행 이력을 조회합니다.")
    @GetMapping
    public ApiResponse<PageResponse<CommandLogResponse>> getAllCommandLogs(
            @RequestParam(defaultValue = "0") int page,
            @RequestParam(defaultValue = "20") int size,
            @RequestParam(required = false) String robotId,
            @RequestParam(required = false) String commandType,
            @RequestParam(required = false) String status) {

        log.info("📋 전체 명령 로그 조회 - Page: {}, Robot: {}, Type: {}, Status: {}", 
            page, robotId, commandType, status);

        Pageable pageable = PageRequest.of(page, size, Sort.by(Sort.Direction.DESC, "sentAt"));
        
        // 필터 조건에 따라 적절한 서비스 메서드 호출
        PageResponse<CommandLogResponse> logs;
        if (robotId != null) {
            logs = commandLogService.getCommandLogsByRobotId(robotId, pageable);
        } else if (commandType != null) {
            logs = commandLogService.getCommandLogsByType(commandType, pageable);
        } else {
            // 기본적으로는 사용자 ID 기준으로 조회 (현재는 임시로 1L 사용)
            logs = commandLogService.getCommandLogsByUserId(1L, pageable);
        }
        
        return ApiResponse.success(SuccessCode.EVENT_LIST_SUCCESS, logs);
    }
}
