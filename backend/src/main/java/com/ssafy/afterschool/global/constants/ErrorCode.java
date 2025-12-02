package com.ssafy.afterschool.global.constants;

import lombok.AllArgsConstructor;
import lombok.Getter;
import org.springframework.http.HttpStatus;

/**
 * 에러 코드와 HTTP 상태, 메시지를 정의하는 열거형
 */
@Getter
@AllArgsConstructor
public enum ErrorCode {

    // 공통 에러 (COMMON)
    INVALID_REQUEST(HttpStatus.BAD_REQUEST, "잘못된 요청입니다"),
    INVALID_REQUEST_PARAMETER(HttpStatus.BAD_REQUEST, "요청 파라미터가 올바르지 않습니다"),
    VALIDATION_FAILED(HttpStatus.BAD_REQUEST, "입력값 검증에 실패했습니다"),
    RESOURCE_NOT_FOUND(HttpStatus.NOT_FOUND, "요청한 리소스를 찾을 수 없습니다"),
    METHOD_NOT_ALLOWED(HttpStatus.METHOD_NOT_ALLOWED, "허용되지 않은 HTTP 메소드입니다"),
    REQUEST_TIMEOUT(HttpStatus.REQUEST_TIMEOUT, "요청 시간이 초과되었습니다"),
    UNSUPPORTED_MEDIA_TYPE(HttpStatus.UNSUPPORTED_MEDIA_TYPE, "지원하지 않는 미디어 타입입니다"),

    // 인증/인가 관련 (AUTH)
    INVALID_CREDENTIALS(HttpStatus.UNAUTHORIZED, "잘못된 로그인 정보입니다"),
    TOKEN_EXPIRED(HttpStatus.UNAUTHORIZED, "토큰이 만료되었습니다"),
    TOKEN_INVALID(HttpStatus.UNAUTHORIZED, "유효하지 않은 토큰입니다"),
    TOKEN_MISSING(HttpStatus.UNAUTHORIZED, "인증 토큰이 누락되었습니다"),
    ACCESS_DENIED(HttpStatus.FORBIDDEN, "접근 권한이 없습니다"),
    ACCOUNT_DISABLED(HttpStatus.FORBIDDEN, "비활성화된 계정입니다"),

    // 사용자 관련 (USER)
    USER_NOT_FOUND(HttpStatus.NOT_FOUND, "사용자를 찾을 수 없습니다"),
    USERNAME_ALREADY_EXISTS(HttpStatus.CONFLICT, "이미 존재하는 사용자명입니다"),
    USER_CREATION_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "사용자 생성에 실패했습니다"),
    USER_UPDATE_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "사용자 정보 수정에 실패했습니다"),
    USER_DELETE_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "사용자 삭제에 실패했습니다"),
    INVALID_PASSWORD_FORMAT(HttpStatus.BAD_REQUEST, "비밀번호 형식이 올바르지 않습니다"),
    CANNOT_DELETE_LAST_ADMIN(HttpStatus.CONFLICT, "마지막 관리자를 삭제할 수 없습니다"),

    // 로봇 관련 (ROBOT)
    ROBOT_NOT_FOUND(HttpStatus.NOT_FOUND, "로봇을 찾을 수 없습니다"),
    ROBOT_ALREADY_PATROLLING(HttpStatus.CONFLICT, "이미 순찰 중인 로봇입니다"),
    ROBOT_NOT_AVAILABLE(HttpStatus.CONFLICT, "사용할 수 없는 로봇 상태입니다"),
    ROBOT_OFFLINE(HttpStatus.CONFLICT, "로봇이 오프라인 상태입니다"),
    ROBOT_LOW_BATTERY(HttpStatus.CONFLICT, "로봇 배터리가 부족합니다"),
    ROBOT_COMMUNICATION_ERROR(HttpStatus.SERVICE_UNAVAILABLE, "로봇 통신 오류가 발생했습니다"),
    ROBOT_COMMAND_TIMEOUT(HttpStatus.GATEWAY_TIMEOUT, "로봇 명령 응답 시간이 초과되었습니다"),
    ROBOT_COMMAND_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "로봇 명령 실행에 실패했습니다"),
    ROBOT_HARDWARE_ERROR(HttpStatus.INTERNAL_SERVER_ERROR, "로봇 하드웨어 오류가 발생했습니다"),
    ROBOT_NAVIGATION_ERROR(HttpStatus.INTERNAL_SERVER_ERROR, "로봇 내비게이션 오류가 발생했습니다"),
    INVALID_ROBOT_COMMAND(HttpStatus.BAD_REQUEST, "유효하지 않은 로봇 명령입니다"),
    ROBOT_EMERGENCY_STOP_ACTIVE(HttpStatus.CONFLICT, "긴급정지 상태에서는 작업을 수행할 수 없습니다"),

    // 맵 관련 (MAP)
    MAP_NOT_FOUND(HttpStatus.NOT_FOUND, "맵을 찾을 수 없습니다"),
    MAP_CREATION_IN_PROGRESS(HttpStatus.CONFLICT, "맵 생성이 이미 진행 중입니다"),
    MAP_CREATION_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "맵 생성에 실패했습니다"),
    MAP_FILE_UPLOAD_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "맵 파일 업로드에 실패했습니다"),
    MAP_FILE_CORRUPTED(HttpStatus.BAD_REQUEST, "손상된 맵 파일입니다"),
    MAP_ACTIVATION_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "맵 활성화에 실패했습니다"),
    MAP_ALREADY_ACTIVE(HttpStatus.CONFLICT, "이미 활성화된 맵입니다"),
    MAP_IN_USE(HttpStatus.CONFLICT, "사용 중인 맵은 삭제할 수 없습니다"),
    INVALID_MAP_FORMAT(HttpStatus.BAD_REQUEST, "유효하지 않은 맵 파일 형식입니다"),
    MAP_SIZE_EXCEEDED(HttpStatus.PAYLOAD_TOO_LARGE, "맵 파일 크기가 제한을 초과했습니다"),

    // 구역 관련 (ZONE)
    ZONE_NOT_FOUND(HttpStatus.NOT_FOUND, "구역을 찾을 수 없습니다"),
    ZONE_ALREADY_ASSIGNED(HttpStatus.CONFLICT, "이미 할당된 구역입니다"),
    ZONE_ASSIGNMENT_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "구역 할당에 실패했습니다"),
    INVALID_ZONE_BOUNDARY(HttpStatus.BAD_REQUEST, "유효하지 않은 구역 경계입니다"),
    ZONE_OVERLAP_DETECTED(HttpStatus.CONFLICT, "구역 겹침이 감지되었습니다"),

    // 이벤트 관련 (EVENT)
    EVENT_NOT_FOUND(HttpStatus.NOT_FOUND, "이벤트를 찾을 수 없습니다"),
    EVENT_PROCESSING_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "이벤트 처리에 실패했습니다"),
    INVALID_EVENT_TYPE(HttpStatus.BAD_REQUEST, "유효하지 않은 이벤트 타입입니다"),
    EVENT_ALREADY_PROCESSED(HttpStatus.CONFLICT, "이미 처리된 이벤트입니다"),

    // 파일 관련 (FILE & S3)
    FILE_NOT_FOUND(HttpStatus.NOT_FOUND, "파일을 찾을 수 없습니다"),
    FILE_DOWNLOAD_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "파일 다운로드에 실패했습니다"),
    FILE_UPLOAD_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "파일 업로드에 실패했습니다"),
    FILE_SIZE_EXCEEDED(HttpStatus.PAYLOAD_TOO_LARGE, "파일 크기가 제한을 초과했습니다"),
    FILE_TYPE_NOT_SUPPORTED(HttpStatus.UNSUPPORTED_MEDIA_TYPE, "지원하지 않는 파일 형식입니다"),
    FILE_CORRUPTED(HttpStatus.BAD_REQUEST, "손상된 파일입니다"),
    INSUFFICIENT_STORAGE(HttpStatus.INSUFFICIENT_STORAGE, "저장 공간이 부족합니다"),
    
    // S3 관련 (S3)
    S3_CONNECTION_ERROR(HttpStatus.SERVICE_UNAVAILABLE, "S3 연결 오류가 발생했습니다"),
    S3_BUCKET_NOT_FOUND(HttpStatus.NOT_FOUND, "S3 버킷을 찾을 수 없습니다"),
    S3_ACCESS_DENIED(HttpStatus.FORBIDDEN, "S3 접근 권한이 없습니다"),
    S3_OPERATION_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "S3 작업에 실패했습니다"),

    // 알림 관련 (NOTIFICATION)
    NOTIFICATION_SEND_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "알림 전송에 실패했습니다"),
    NOTIFICATION_NOT_FOUND(HttpStatus.NOT_FOUND, "알림을 찾을 수 없습니다"),
    WEBSOCKET_CONNECTION_ERROR(HttpStatus.SERVICE_UNAVAILABLE, "WebSocket 연결 오류가 발생했습니다"),
    EMAIL_SEND_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "이메일 전송에 실패했습니다"),

    // MQTT 관련 (MQTT)
    MQTT_CONNECTION_ERROR(HttpStatus.SERVICE_UNAVAILABLE, "MQTT 브로커 연결 오류가 발생했습니다"),
    MQTT_PUBLISH_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "MQTT 메시지 발행에 실패했습니다"),
    MQTT_SUBSCRIBE_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "MQTT 토픽 구독에 실패했습니다"),
    MQTT_TOPIC_INVALID(HttpStatus.BAD_REQUEST, "유효하지 않은 MQTT 토픽입니다"),
    MQTT_PAYLOAD_INVALID(HttpStatus.BAD_REQUEST, "유효하지 않은 MQTT 페이로드입니다"),
    MQTT_MESSAGE_PARSING_FAILED(HttpStatus.BAD_REQUEST, "MQTT 메시지 파싱에 실패했습니다"),
    MQTT_CHANNEL_UNAVAILABLE(HttpStatus.SERVICE_UNAVAILABLE, "MQTT 채널을 사용할 수 없습니다"),
    MQTT_CLIENT_ERROR(HttpStatus.INTERNAL_SERVER_ERROR, "MQTT 클라이언트 오류가 발생했습니다"),
    MQTT_BROKER_UNAVAILABLE(HttpStatus.SERVICE_UNAVAILABLE, "MQTT 브로커에 연결할 수 없습니다"),
    MQTT_QOS_NOT_SUPPORTED(HttpStatus.BAD_REQUEST, "지원하지 않는 MQTT QoS 수준입니다"),

    // 시스템 관련 (SYSTEM)
    INTERNAL_SERVER_ERROR(HttpStatus.INTERNAL_SERVER_ERROR, "내부 서버 오류가 발생했습니다"),
    SERVICE_UNAVAILABLE(HttpStatus.SERVICE_UNAVAILABLE, "서비스를 이용할 수 없습니다"),
    DATABASE_CONNECTION_ERROR(HttpStatus.SERVICE_UNAVAILABLE, "데이터베이스 연결 오류가 발생했습니다"),
    DATABASE_OPERATION_FAILED(HttpStatus.INTERNAL_SERVER_ERROR, "데이터베이스 작업에 실패했습니다"),
    CONFIGURATION_ERROR(HttpStatus.INTERNAL_SERVER_ERROR, "시스템 설정 오류가 발생했습니다"),
    EXTERNAL_SERVICE_ERROR(HttpStatus.BAD_GATEWAY, "외부 서비스 오류가 발생했습니다"),
    MAINTENANCE_MODE(HttpStatus.SERVICE_UNAVAILABLE, "시스템 점검 중입니다");

    private final HttpStatus status;
    private final String message;
}