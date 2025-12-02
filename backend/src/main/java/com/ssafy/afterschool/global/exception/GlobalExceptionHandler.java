package com.ssafy.afterschool.global.exception;

import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.TypeMismatchException;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatusCode;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.HttpMessageNotReadableException;
import org.springframework.web.HttpMediaTypeNotAcceptableException;
import org.springframework.web.HttpMediaTypeNotSupportedException;
import org.springframework.web.HttpRequestMethodNotSupportedException;
import org.springframework.web.bind.MethodArgumentNotValidException;
import org.springframework.web.bind.MissingPathVariableException;
import org.springframework.web.bind.MissingServletRequestParameterException;
import org.springframework.web.bind.ServletRequestBindingException;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;
import org.springframework.web.context.request.WebRequest;
import org.springframework.web.method.annotation.HandlerMethodValidationException;
import org.springframework.web.servlet.NoHandlerFoundException;
import org.springframework.web.servlet.mvc.method.annotation.ResponseEntityExceptionHandler;
import org.springframework.web.servlet.resource.NoResourceFoundException;

/**
 * 전역 예외 처리를 담당하는 핸들러 클래스
 *
 * 처리 우선순위:
 * 1. ApiException (비즈니스 로직 예외)
 * 2. Spring 내장 예외들 (Validation, HTTP 관련)
 * 3. 일반 예외 (예상하지 못한 예외)
 */
@Slf4j
@RestControllerAdvice
public class GlobalExceptionHandler extends ResponseEntityExceptionHandler {

    // ================================
    // 1. 커스텀 비즈니스 예외 처리
    // ================================

    /**
     * 비즈니스 로직에서 발생하는 예외 처리 (가장 우선)
     *
     * @param e ApiException
     * @return 에러 응답
     */
    @ExceptionHandler(ApiException.class)
    public ResponseEntity<ApiResponse<Void>> handleApiException(ApiException e) {
        ErrorCode errorCode = e.getErrorCode();

        // 로그 레벨 구분
        if (errorCode.getStatus().is5xxServerError()) {
            log.error("API error occurred: {} - {}", errorCode.name(), e.getMessage(), e);
        } else {
            log.warn("API error occurred: {} - {}", errorCode.name(), e.getMessage());
        }

        return ResponseEntity.status(errorCode.getStatus())
                .body(ApiResponse.fail(errorCode));
    }

    // ================================
    // 2. Spring 내장 예외 처리 (Override)
    // ================================

    /**
     * Spring에서 발생하는 내장 예외들 처리
     * ResponseEntityExceptionHandler의 메서드를 오버라이드
     */
    @Override
    protected ResponseEntity<Object> handleExceptionInternal(
            Exception e,
            Object body,
            HttpHeaders headers,
            HttpStatusCode statusCode,
            WebRequest request) {

        ErrorCode errorCode = mapToErrorCode(e);
        log.warn("Spring error occurred: {} - {}", errorCode.name(), e.getMessage());

        return ResponseEntity.status(errorCode.getStatus())
                .body(ApiResponse.fail(errorCode));
    }

    /**
     * Bean Validation 실패 시 처리 (@Valid 애노테이션)
     */
    @Override
    protected ResponseEntity<Object> handleMethodArgumentNotValid(
            MethodArgumentNotValidException e,
            HttpHeaders headers,
            HttpStatusCode status,
            WebRequest request) {

        log.warn("Validation error occurred: {}", e.getMessage());

        // 첫 번째 필드 에러 메시지 추출
        String fieldError = e.getBindingResult().getFieldErrors().stream()
                .findFirst()
                .map(error -> error.getField() + ": " + error.getDefaultMessage())
                .orElse("입력값 검증에 실패했습니다");

        log.debug("Validation details: {}", fieldError);

        return ResponseEntity.badRequest()
                .body(ApiResponse.fail(ErrorCode.VALIDATION_FAILED));
    }

    // ================================
    // 3. 일반 예외 처리 (최종 안전망)
    // ================================

    /**
     * 처리되지 않은 모든 예외에 대한 최종 핸들러
     *
     * @param e 예상하지 못한 예외
     * @return 내부 서버 오류 응답
     */
    @ExceptionHandler(Exception.class)
    public ResponseEntity<ApiResponse<Void>> handleGeneralException(Exception e) {
        log.error("Unexpected error occurred: {}", e.getMessage(), e);

        return ResponseEntity.status(ErrorCode.INTERNAL_SERVER_ERROR.getStatus())
                .body(ApiResponse.fail(ErrorCode.INTERNAL_SERVER_ERROR));
    }

    // ================================
    // 4. Spring 예외 → ErrorCode 매핑
    // ================================

    /**
     * Spring 내장 예외를 적절한 ErrorCode로 매핑
     *
     * @param e Spring 예외
     * @return 해당하는 ErrorCode
     */
    private ErrorCode mapToErrorCode(Exception e) {
        // 400 Bad Request 계열
        if (e instanceof MethodArgumentNotValidException
                || e instanceof HttpMessageNotReadableException
                || e instanceof HandlerMethodValidationException
                || e instanceof MissingServletRequestParameterException
                || e instanceof TypeMismatchException
                || e instanceof ServletRequestBindingException
                || e instanceof MissingPathVariableException) {
            return ErrorCode.INVALID_REQUEST;
        }

        // 405 Method Not Allowed
        if (e instanceof HttpRequestMethodNotSupportedException) {
            return ErrorCode.METHOD_NOT_ALLOWED;
        }

        // 415 Unsupported Media Type
        if (e instanceof HttpMediaTypeNotSupportedException) {
            return ErrorCode.UNSUPPORTED_MEDIA_TYPE;
        }

        // 406 Not Acceptable
        if (e instanceof HttpMediaTypeNotAcceptableException) {
            return ErrorCode.INVALID_REQUEST;
        }

        // 404 Not Found
        if (e instanceof NoHandlerFoundException
                || e instanceof NoResourceFoundException) {
            return ErrorCode.RESOURCE_NOT_FOUND;
        }

        // 기본값: 500 Internal Server Error
        return ErrorCode.INTERNAL_SERVER_ERROR;
    }

    // ================================
    // 사용 예시
    // ================================

    /*
     * 처리되는 예외들:
     *
     * 1. ApiException
     *    - throw new ApiException(ErrorCode.USER_NOT_FOUND);
     *    → HTTP 404 + {"status": "ERROR", "message": "사용자를 찾을 수 없습니다", "data": null}
     *
     * 2. Validation 에러
     *    - @Valid 실패
     *    → HTTP 400 + {"status": "ERROR", "message": "입력값 검증에 실패했습니다", "data": null}
     *
     * 3. Spring 내장 예외
     *    - 잘못된 JSON 형식
     *    → HTTP 400 + {"status": "ERROR", "message": "잘못된 요청입니다", "data": null}
     *
     * 4. 예상하지 못한 예외
     *    - NullPointerException 등
     *    → HTTP 500 + {"status": "ERROR", "message": "내부 서버 오류가 발생했습니다", "data": null}
     */
}