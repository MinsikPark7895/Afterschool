package com.ssafy.afterschool.domain.robot.service;

import com.ssafy.afterschool.domain.robot.dto.*;
import com.ssafy.afterschool.domain.robot.entity.Robot;

import java.util.List;

/**
 * 로봇 서비스 인터페이스
 * - 로봇 관련 비즈니스 로직 정의
 */
public interface RobotService {

    /**
     * 모든 로봇 목록 조회
     * @return 로봇 정보 목록
     */
    List<RobotInfoResponse> getAllRobots();

    /**
     * 특정 로봇 조회
     * @param robotId 로봇 식별자
     * @return 로봇 정보
     */
    RobotInfoResponse getRobotById(String robotId);

    /**
     * 로봇 이동 명령
     * @param robotId 로봇 식별자
     * @param request 이동 요청 정보
     * @return 명령 실행 응답
     */
    CommandResponse moveRobot(String robotId, MoveToRequest request);

    /**
     * 순찰 시작 명령 (일괄)
     * @return 명령 실행 응답
     */
    CommandResponse startPatrol();

    /**
     * 순찰 중지 명령
     * @return 명령 실행 응답
     */
    CommandResponse stopPatrol();
}
