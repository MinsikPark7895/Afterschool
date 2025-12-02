package com.ssafy.afterschool.domain.robot.repository;

import com.ssafy.afterschool.domain.robot.entity.Robot;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

/**
 * 로봇 리포지토리
 * - 로봇 정보 CRUD 및 조회 기능
 */
@Repository
public interface RobotRepository extends JpaRepository<Robot, Long> {

    /**
     * 로봇 식별자로 로봇 조회
     * @param robotId ROS2에서 사용하는 로봇 식별자 ("tb1", "tb2")
     * @return 로봇 정보
     */
    Optional<Robot> findByRobotId(String robotId);

    /**
     * 활성화된 로봇 목록 조회
     * @param isActive 활성화 상태
     * @return 활성화된 로봇 목록
     */
    List<Robot> findByIsActive(boolean isActive);
}
