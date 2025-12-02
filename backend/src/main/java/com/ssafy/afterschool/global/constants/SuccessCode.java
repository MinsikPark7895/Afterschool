package com.ssafy.afterschool.global.constants;

import lombok.AllArgsConstructor;
import lombok.Getter;

/**
 * 성공 응답 메시지를 정의하는 열거형
 */
@Getter
@AllArgsConstructor
public enum SuccessCode {

    // 공통
    SUCCESS_DEFAULT("요청이 성공적으로 처리되었습니다"),

    // 인증 관련
    LOGIN_SUCCESS("로그인이 성공했습니다"),
    LOGOUT_SUCCESS("로그아웃이 완료되었습니다"),
    TOKEN_REFRESH_SUCCESS("토큰 갱신이 완료되었습니다"),

    // 사용자 관련
    USER_PROFILE_SUCCESS("프로필 조회가 완료되었습니다"),
    USER_LIST_SUCCESS("사용자 목록 조회가 완료되었습니다"),
    USER_CREATE_SUCCESS("사용자가 성공적으로 생성되었습니다"),
    USER_UPDATE_SUCCESS("사용자 정보가 성공적으로 수정되었습니다"),
    USER_DELETE_SUCCESS("사용자가 성공적으로 삭제되었습니다"),

    // 로봇 관련
    ROBOT_LIST_SUCCESS("로봇 목록 조회가 완료되었습니다"),
    ROBOT_DETAIL_SUCCESS("로봇 정보 조회가 완료되었습니다"),
    ROBOT_UPDATE_SUCCESS("로봇 설정이 성공적으로 수정되었습니다"),
    PATROL_START_SUCCESS("순찰이 성공적으로 시작되었습니다"),
    PATROL_STOP_SUCCESS("순찰이 성공적으로 중지되었습니다"),
    ROBOT_MOVE_SUCCESS("이동 명령이 성공적으로 전송되었습니다"),
    EMERGENCY_STOP_SUCCESS("긴급정지 명령이 모든 로봇에 전송되었습니다"),

    // 맵 관련
    MAP_LIST_SUCCESS("맵 목록 조회가 완료되었습니다"),
    MAP_CREATE_SUCCESS("맵 생성이 시작되었습니다"),
    MAP_SAVE_SUCCESS("맵이 성공적으로 저장되었습니다"),
    MAP_DETAIL_SUCCESS("맵 상세 정보 조회가 완료되었습니다"),
    MAP_ACTIVATE_SUCCESS("맵이 성공적으로 활성화되었습니다"),

    // 구역 관련
    ZONE_LIST_SUCCESS("구역 목록 조회가 완료되었습니다"),

    // 이벤트 관련
    EVENT_LIST_SUCCESS("이벤트 목록 조회가 완료되었습니다"),
    EVENT_DETAIL_SUCCESS("이벤트 상세 정보 조회가 완료되었습니다"),
    EVENT_UPDATE_SUCCESS("이벤트가 성공적으로 처리되었습니다"),

    // 파일 관련
    FILE_DOWNLOAD_SUCCESS("파일 다운로드가 완료되었습니다");

    private final String message;
}
