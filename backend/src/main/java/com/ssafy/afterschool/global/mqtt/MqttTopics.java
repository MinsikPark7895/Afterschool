package com.ssafy.afterschool.global.mqtt;

import lombok.experimental.UtilityClass;

/**
 * MQTT 토픽 패턴 상수 클래스
 * - 모든 MQTT 토픽 패턴을 중앙 집중 관리
 * - 토픽 변경 시 한 곳에서만 수정 가능
 */
@UtilityClass
public class MqttTopics {

    // ======================== FROM ROBOT 토픽 (로봇 → 서버) ========================

    /**
     * 로봇 기본 상태 정보 토픽
     * - 패턴: from_robot/status/basic/{robot_id}
     * - 용도: 위치, 배터리, 연결 상태 등 기본 정보
     */
    public static final String FROM_ROBOT_STATUS_BASIC = "from_robot/status/basic/";

    /**
     * 로봇 상세 상태 정보 토픽
     * - 패턴: from_robot/status/detail/{robot_id}
     * - 용도: 센서 데이터, 하드웨어 상태 등 상세 정보
     */
    public static final String FROM_ROBOT_STATUS_DETAIL = "from_robot/status/detail/";

    /**
     * 로봇 이벤트 감지 토픽
     * - 패턴: from_robot/event/detection/{robot_id}
     * - 용도: 침입자 감지, 이상 상황 감지 등
     */
    public static final String FROM_ROBOT_EVENT_DETECTION = "from_robot/event/detection/";

    /**
     * 로봇 미션 완료 토픽
     * - 패턴: from_robot/event/mission_done/{robot_id}
     * - 용도: 순찰 완료, 이동 완료 등 미션 결과
     */
    public static final String FROM_ROBOT_EVENT_MISSION = "from_robot/event/mission_done/";

    /**
     * 시스템 상태 토픽
     * - 패턴: from_robot/system/event/system_status
     * - 용도: 전체 시스템 상태, 네트워크 상태 등
     */
    public static final String FROM_ROBOT_SYSTEM_STATUS = "from_robot/system/event/system_status";

    // ======================== TO ROBOT 토픽 (서버 → 로봇) ========================

    /**
     * 로봇 명령 토픽 베이스
     * - 개별 명령: to_robot/command/{command_type}/{robot_id}
     * - 일괄 명령: to_robot/command/{command_type}
     */
    public static final String TO_ROBOT_COMMAND = "to_robot/command/";

    // ======================== 토픽 빌더 메서드 ========================

    /**
     * 로봇 기본 상태 토픽 생성
     * @param robotId 로봇 ID
     * @return 완성된 토픽
     */
    public static String robotStatusBasic(String robotId) {
        return FROM_ROBOT_STATUS_BASIC + robotId;
    }

    /**
     * 로봇 상세 상태 토픽 생성
     * @param robotId 로봇 ID
     * @return 완성된 토픽
     */
    public static String robotStatusDetail(String robotId) {
        return FROM_ROBOT_STATUS_DETAIL + robotId;
    }

    /**
     * 로봇 이벤트 감지 토픽 생성
     * @param robotId 로봇 ID
     * @return 완성된 토픽
     */
    public static String robotEventDetection(String robotId) {
        return FROM_ROBOT_EVENT_DETECTION + robotId;
    }

    /**
     * 로봇 미션 완료 토픽 생성
     * @param robotId 로봇 ID
     * @return 완성된 토픽
     */
    public static String robotEventMission(String robotId) {
        return FROM_ROBOT_EVENT_MISSION + robotId;
    }

    /**
     * 개별 로봇 명령 토픽 생성
     * @param commandType 명령 타입 (move_to 등)
     * @param robotId 로봇 ID
     * @return 완성된 토픽
     */
    public static String robotCommand(String commandType, String robotId) {
        return TO_ROBOT_COMMAND + commandType + "/" + robotId;
    }

    /**
     * 일괄 로봇 명령 토픽 생성
     * @param commandType 명령 타입 (CommandType 상수 사용)
     * @return 완성된 토픽
     */
    public static String robotCommand(String commandType) {
        return TO_ROBOT_COMMAND + commandType;
    }
}