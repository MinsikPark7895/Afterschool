package com.ssafy.afterschool.domain.event.service;

import com.ssafy.afterschool.domain.event.dto.CommandLogResponse;
import com.ssafy.afterschool.domain.event.entity.CommandLog;
import com.ssafy.afterschool.global.response.PageResponse;
import org.springframework.data.domain.Pageable;

import java.time.LocalDateTime;
import java.util.List;

/**
 * 명령 로그 서비스 인터페이스
 */
public interface CommandLogService {

    /**
     * 명령 로그 생성
     */
    CommandLog createCommandLog(String targetRobotId, Long userId, String commandType, String commandPayload);

    /**
     * 명령 완료 처리
     */
    void markCommandAsCompleted(Long commandId, String responseData);

    /**
     * 명령 실패 처리
     */
    void markCommandAsFailed(Long commandId, String errorResponse);

    /**
     * 사용자별 명령 이력 조회
     */
    PageResponse<CommandLogResponse> getCommandLogsByUserId(Long userId, Pageable pageable);

    /**
     * 로봇별 명령 이력 조회
     */
    PageResponse<CommandLogResponse> getCommandLogsByRobotId(String robotId, Pageable pageable);

    /**
     * 명령 타입별 조회
     */
    PageResponse<CommandLogResponse> getCommandLogsByType(String commandType, Pageable pageable);


    /**
     * 간단한 명령 로그 생성 (AOP용)
     */
    Long logCommand(String robotId, String commandType, String commandPayload, com.ssafy.afterschool.domain.event.entity.CommandStatus status, Long userId);

    /**
     * 명령 상태 업데이트 (AOP용)
     */
    void updateCommandStatus(Long commandLogId, com.ssafy.afterschool.domain.event.entity.CommandStatus status);
}
