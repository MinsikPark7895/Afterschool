package com.ssafy.afterschool.domain.event.service;

import com.ssafy.afterschool.domain.event.dto.CommandLogResponse;
import com.ssafy.afterschool.domain.event.entity.CommandLog;
import com.ssafy.afterschool.domain.event.repository.CommandLogRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.response.PageResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

/**
 * 명령 로그 서비스 구현체
 */
@Slf4j
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class CommandLogServiceImpl implements CommandLogService {

    private final CommandLogRepository commandLogRepository;

    @Override
    @Transactional
    public CommandLog createCommandLog(String targetRobotId, Long userId, String commandType, String commandPayload) {
        CommandLog commandLog = CommandLog.builder()
            .targetRobotId(targetRobotId)
            .userId(userId)
            .commandType(commandType)
            .commandPayload(commandPayload)
            .commandStatus("sent")
            .build();

        CommandLog saved = commandLogRepository.save(commandLog);
        log.info("📝 명령 로그 생성 완료 - ID: {}, Type: {}, Robot: {}", 
            saved.getId(), commandType, targetRobotId);

        return saved;
    }

    @Override
    @Transactional
    public void markCommandAsCompleted(Long commandId, String responseData) {
        CommandLog commandLog = commandLogRepository.findById(commandId)
            .orElseThrow(() -> new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "명령 로그를 찾을 수 없습니다."));

        commandLog.markAsCompleted(responseData);
        commandLogRepository.save(commandLog);

        log.info("✅ 명령 완료 처리 - ID: {}, Type: {}", commandId, commandLog.getCommandType());
    }

    @Override
    @Transactional
    public void markCommandAsFailed(Long commandId, String errorResponse) {
        CommandLog commandLog = commandLogRepository.findById(commandId)
            .orElseThrow(() -> new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "명령 로그를 찾을 수 없습니다."));

        commandLog.markAsFailed(errorResponse);
        commandLogRepository.save(commandLog);

        log.error("❌ 명령 실패 처리 - ID: {}, Type: {}", commandId, commandLog.getCommandType());
    }

    @Override
    public PageResponse<CommandLogResponse> getCommandLogsByUserId(Long userId, Pageable pageable) {
        Page<CommandLog> commandLogPage = commandLogRepository.findByUserIdOrderBySentAtDesc(userId, pageable);
        List<CommandLogResponse> responses = commandLogPage.getContent().stream()
            .map(this::convertToCommandLogResponse)
            .collect(Collectors.toList());

        return PageResponse.of(commandLogPage, responses);
    }

    @Override
    public PageResponse<CommandLogResponse> getCommandLogsByRobotId(String robotId, Pageable pageable) {
        Page<CommandLog> commandLogPage = commandLogRepository.findByTargetRobotIdOrderBySentAtDesc(robotId, pageable);
        List<CommandLogResponse> responses = commandLogPage.getContent().stream()
            .map(this::convertToCommandLogResponse)
            .collect(Collectors.toList());

        return PageResponse.of(commandLogPage, responses);
    }

    @Override
    public PageResponse<CommandLogResponse> getCommandLogsByType(String commandType, Pageable pageable) {
        Page<CommandLog> commandLogPage = commandLogRepository.findByCommandTypeOrderBySentAtDesc(commandType, pageable);
        List<CommandLogResponse> responses = commandLogPage.getContent().stream()
            .map(this::convertToCommandLogResponse)
            .collect(Collectors.toList());

        return PageResponse.of(commandLogPage, responses);
    }


    @Override
    @Transactional
    public Long logCommand(String robotId, String commandType, String commandPayload, com.ssafy.afterschool.domain.event.entity.CommandStatus status, Long userId) {
        CommandLog commandLog = CommandLog.builder()
            .targetRobotId(robotId)
            .userId(userId)
            .commandType(commandType)
            .commandPayload(commandPayload)
            .commandStatus(status.name().toLowerCase())
            .build();

        CommandLog saved = commandLogRepository.save(commandLog);
        log.info("📝 명령 로그 생성 (AOP) - ID: {}, Robot: {}, Type: {}, Status: {}", 
            saved.getId(), robotId, commandType, status);

        return saved.getId();
    }

    @Override
    @Transactional
    public void updateCommandStatus(Long commandLogId, com.ssafy.afterschool.domain.event.entity.CommandStatus status) {
        CommandLog commandLog = commandLogRepository.findById(commandLogId)
            .orElseThrow(() -> new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "명령 로그를 찾을 수 없습니다."));

        commandLog.updateStatus(status.name().toLowerCase());
        
        if (status == com.ssafy.afterschool.domain.event.entity.CommandStatus.SUCCESS) {
            commandLog.markAsCompleted(null);
        } else if (status == com.ssafy.afterschool.domain.event.entity.CommandStatus.FAILED) {
            commandLog.markAsFailed("Command execution failed");
        }
        
        commandLogRepository.save(commandLog);
        log.info("🔄 명령 상태 업데이트 (AOP) - ID: {}, Status: {}", commandLogId, status);
    }

    // === Private Methods ===

    private CommandLogResponse convertToCommandLogResponse(CommandLog commandLog) {
        Long executionTime = null;
        if (commandLog.getCompletedAt() != null) {
            executionTime = Duration.between(commandLog.getSentAt(), commandLog.getCompletedAt()).getSeconds();
        }

        return CommandLogResponse.builder()
            .id(commandLog.getId())
            .targetRobotId(commandLog.getTargetRobotId())
            .userId(commandLog.getUserId())
            .commandType(commandLog.getCommandType())
            .commandPayload(commandLog.getCommandPayload())
            .commandStatus(commandLog.getCommandStatus())
            .responseData(commandLog.getResponseData())
            .sentAt(commandLog.getSentAt())
            .completedAt(commandLog.getCompletedAt())
            .executionTime(executionTime)
            .build();
    }
}
