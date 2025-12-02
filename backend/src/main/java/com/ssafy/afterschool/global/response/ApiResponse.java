package com.ssafy.afterschool.global.response;

import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Status;
import com.ssafy.afterschool.global.constants.SuccessCode;

/**
 * API 공통 응답 형식을 정의하는 Record 클래스
 *
 * @param status 응답 상태 (SUCCESS, ERROR)
 * @param message 응답 메시지
 * @param data 응답 데이터
 */
public record ApiResponse<T>(
        Status status,
        String message,
        T data
) {

    // ================================
    // 성공 응답 생성 메서드들
    // ================================

    /**
     * 성공 응답 생성 (SuccessCode + 데이터)
     */
    public static <T> ApiResponse<T> success(SuccessCode successCode, T data) {
        return new ApiResponse<>(Status.SUCCESS, successCode.getMessage(), data);
    }

    /**
     * 성공 응답 생성 (데이터만, 기본 메시지 사용)
     */
    public static <T> ApiResponse<T> success(T data) {
        return new ApiResponse<>(Status.SUCCESS, SuccessCode.SUCCESS_DEFAULT.getMessage(), data);
    }

    /**
     * 성공 응답 생성 (SuccessCode만, 데이터 없음)
     */
    public static ApiResponse<Void> success(SuccessCode successCode) {
        return new ApiResponse<>(Status.SUCCESS, successCode.getMessage(), null);
    }

    /**
     * 성공 응답 생성 (기본 메시지, 데이터 없음)
     */
    public static ApiResponse<Void> success() {
        return new ApiResponse<>(Status.SUCCESS, SuccessCode.SUCCESS_DEFAULT.getMessage(), null);
    }

    // ================================
    // 실패 응답 생성 메서드들
    // ================================

    /**
     * 실패 응답 생성 (ErrorCode + 데이터)
     */
    public static <T> ApiResponse<T> fail(ErrorCode errorCode, T data) {
        return new ApiResponse<>(Status.ERROR, errorCode.getMessage(), data);
    }

    /**
     * 실패 응답 생성 (ErrorCode만, 데이터 없음)
     */
    public static ApiResponse<Void> fail(ErrorCode errorCode) {
        return new ApiResponse<>(Status.ERROR, errorCode.getMessage(), null);
    }

    // ================================
    // 사용 예시
    // ================================

    /*
     * 사용 예시:
     *
     * // 성공 응답 (데이터 포함)
     * return ApiResponse.success(SuccessCode.LOGIN_SUCCESS, userData);
     *
     * // 성공 응답 (데이터 없음)
     * return ApiResponse.success(SuccessCode.LOGOUT_SUCCESS);
     *
     * // 실패 응답
     * return ApiResponse.fail(ErrorCode.USER_NOT_FOUND);
     */
}