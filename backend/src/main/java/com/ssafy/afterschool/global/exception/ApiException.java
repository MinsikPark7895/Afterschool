package com.ssafy.afterschool.global.exception;

import com.ssafy.afterschool.global.constants.ErrorCode;
import lombok.Getter;

/**
 * 비즈니스 로직에서 발생하는 예외를 처리하기 위한 커스텀 예외 클래스
 *
 * 사용법:
 * - throw new ApiException(ErrorCode.USER_NOT_FOUND);
 * - throw new ApiException(ErrorCode.ROBOT_NOT_FOUND, "robot_id: " + robotId);
 */
@Getter
public class ApiException extends RuntimeException {

    private static final long serialVersionUID = 1L;

    /**
     * 에러 코드 (HTTP 상태와 메시지 포함)
     */
    private final ErrorCode errorCode;

    /**
     * 추가 상세 정보 (선택사항)
     */
    private final String details;

    /**
     * 기본 생성자 - ErrorCode만 사용
     *
     * @param errorCode 에러 코드
     */
    public ApiException(ErrorCode errorCode) {
        super(errorCode.getMessage());
        this.errorCode = errorCode;
        this.details = null;
    }

    /**
     * 상세 정보 포함 생성자
     *
     * @param errorCode 에러 코드
     * @param details 상세 정보 (예: "robot_id: tb1")
     */
    public ApiException(ErrorCode errorCode, String details) {
        super(errorCode.getMessage() + (details != null ? " - " + details : ""));
        this.errorCode = errorCode;
        this.details = details;
    }

    /**
     * 원인 예외 포함 생성자
     *
     * @param errorCode 에러 코드
     * @param cause 원인 예외
     */
    public ApiException(ErrorCode errorCode, Throwable cause) {
        super(errorCode.getMessage(), cause);
        this.errorCode = errorCode;
        this.details = null;
    }

    /**
     * 상세 정보 + 원인 예외 포함 생성자
     *
     * @param errorCode 에러 코드
     * @param details 상세 정보
     * @param cause 원인 예외
     */
    public ApiException(ErrorCode errorCode, String details, Throwable cause) {
        super(errorCode.getMessage() + (details != null ? " - " + details : ""), cause);
        this.errorCode = errorCode;
        this.details = details;
    }

    // ================================
    // 사용 예시
    // ================================

    /*
     * 사용 예시:
     *
     * // 기본 사용
     * throw new ApiException(ErrorCode.USER_NOT_FOUND);
     *
     * // 상세 정보 포함
     * throw new ApiException(ErrorCode.ROBOT_NOT_FOUND, "robot_id: " + robotId);
     *
     * // 원인 예외 포함 (DB 예외 등을 래핑할 때)
     * throw new ApiException(ErrorCode.DATABASE_CONNECTION_ERROR, sqlException);
     *
     * // Service에서 사용 예시:
     * public User findUserById(Long userId) {
     *     return userRepository.findById(userId)
     *         .orElseThrow(() -> new ApiException(ErrorCode.USER_NOT_FOUND, "userId: " + userId));
     * }
     */
}