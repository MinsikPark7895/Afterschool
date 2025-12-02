package com.ssafy.afterschool.global.aspect;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.afterschool.domain.event.entity.CommandStatus;
import com.ssafy.afterschool.domain.event.service.CommandLogService;
import com.ssafy.afterschool.global.security.UserPrincipal;
import com.ssafy.afterschool.global.websocket.WebSocketService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.stereotype.Component;

/**
 * 로봇 명령 로그 자동 기록 AOP
 * - @LogRobotCommand 어노테이션이 붙은 메서드의 명령을 자동 기록
 * - CommandLog 테이블에 저장 + WebSocket으로 실시간 알림
 */
@Slf4j
@Aspect
@Component
@RequiredArgsConstructor
public class CommandLogAspect {

    private final CommandLogService commandLogService;
    private final WebSocketService webSocketService;
    private final ObjectMapper objectMapper;

    @Around("@annotation(logRobotCommand)")
    public Object logCommand(ProceedingJoinPoint joinPoint, LogRobotCommand logRobotCommand) throws Throwable {
        if (!logRobotCommand.enabled()) {
            return joinPoint.proceed();
        }

        Object[] args = joinPoint.getArgs();
        String robotId = extractRobotId(args, joinPoint);
        String commandType = logRobotCommand.commandType();
        Object request = extractRequest(args);
        Long userId = getCurrentUserId();

        Long commandLogId = null;

        try {
            // 1. 명령 시작 로그 기록
            String requestJson = request != null ? objectMapper.writeValueAsString(request) : "{}";
            commandLogId = commandLogService.logCommand(robotId, commandType, requestJson, CommandStatus.SENT, userId);

            log.info("📝 로봇 명령 로그 기록 시작 - ID: {}, Robot: {}, Command: {}",
                commandLogId, robotId, commandType);

            // 2. WebSocket으로 명령 시작 알림
            webSocketService.sendCommandResult(robotId, commandType, "SENT", null);

            // 3. 실제 메서드 실행
            Object result = joinPoint.proceed();

            // 4. 성공 로그 업데이트
            if (commandLogId != null) {
                commandLogService.updateCommandStatus(commandLogId, CommandStatus.SUCCESS);
                log.info("✅ 로봇 명령 실행 성공 - ID: {}, Robot: {}, Command: {}",
                    commandLogId, robotId, commandType);

                // WebSocket으로 성공 결과 전송
                webSocketService.sendCommandResult(robotId, commandType, "SUCCESS", result);
            }

            return result;

        } catch (Exception e) {
            // 5. 실패 로그 업데이트
            if (commandLogId != null) {
                try {
                    commandLogService.updateCommandStatus(commandLogId, CommandStatus.FAILED);
                    log.error("❌ 로봇 명령 실행 실패 - ID: {}, Robot: {}, Command: {}",
                        commandLogId, robotId, commandType, e);

                    // WebSocket으로 실패 결과 전송
                    webSocketService.sendCommandResult(robotId, commandType, "FAILED", e.getMessage());

                } catch (Exception logException) {
                    log.error("❌ 명령 실패 로그 기록 중 오류", logException);
                }
            }

            throw e;
        }
    }

    /**
     * 메서드 인자에서 robotId 추출
     */
    private String extractRobotId(Object[] args, ProceedingJoinPoint joinPoint) {
        // 일괄 명령은 NULL로 처리 (테이블 설계에 따라)
        String methodName = joinPoint.getSignature().getName();
        if ("startPatrol".equals(methodName) || "stopPatrol".equals(methodName)) {
            return null; // NULL이면 일괄/시스템 명령
        }

        // 개별 로봇 명령은 첫 번째 인자에서 robotId 추출
        if (args.length > 0 && args[0] instanceof String) {
            return (String) args[0];
        }
        return "unknown";
    }

    /**
     * 메서드 인자에서 Request DTO 추출
     */
    private Object extractRequest(Object[] args) {
        // 두 번째 인자가 Request DTO인 경우가 대부분
        if (args.length > 1) {
            return args[1];
        }
        return null;
    }

    /**
     * 현재 로그인된 사용자 ID 추출
     */
    private Long getCurrentUserId() {
        try {
            Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
            if (authentication != null && authentication.getPrincipal() instanceof UserPrincipal userPrincipal) {
                return userPrincipal.getId();
            }
        } catch (Exception e) {
            log.warn("⚠️ 사용자 ID 추출 실패", e);
        }
        // 기본값: 시스템 관리자 (ID: 1)
        return 1L;
    }
}
