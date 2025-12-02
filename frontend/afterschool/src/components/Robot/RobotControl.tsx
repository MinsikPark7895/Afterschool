import React, { useState } from "react";
import { robotService } from "../../services/robotService";
import {
  PatrolStartRequest,
  PatrolStopRequest,
  Waypoint,
} from "../../types/robot";

interface RobotControlProps {
  robotId: string;
  currentStatus: string;
  onCommandSent?: (commandType: string) => void;
}

const RobotControl: React.FC<RobotControlProps> = ({
  robotId,
  currentStatus,
  onCommandSent,
}) => {
  const [loading, setLoading] = useState(false);
  
  // 기본 웨이포인트 (나중에 사용자 입력으로 변경 가능)
  // 한마디로 변경해야 함
  const defaultWaypoints: Waypoint[] = [
    { x: 10.0, y: 5.0, order: 1 },
    { x: 15.0, y: 8.0, order: 2 },
    { x: 12.0, y: 12.0, order: 3 }
  ];

  const handleStartPatrol = async () => {
    setLoading(true);
    try {
      const patrolData: PatrolStartRequest = {
        patrolZone: "zone_a", // 실제로는 사용자 입력 받아야 함
        patrolMode: "auto",
        waypoints: defaultWaypoints,  // 나중에 변경해야 함
        patrolSpeed: 0.5,
        loopCount: -1,
        detectionEnabled: true,
        startImmediately: true,
        patrolDuration: 3600,
      };

      await robotService.startPatrol(robotId, patrolData);
      onCommandSent?.("start_patrol");
      alert("순찰이 시작되었습니다.");
    } catch (error) {
      console.error("순찰 시작 실패", error);
      alert("순찰 시작에 실패했습니다.");
    } finally {
      setLoading(false);
    }
  };

  const handleStopPatrol = async () => {
    setLoading(true);
    try {
      const stopData: PatrolStopRequest = {
        stopMode: "immediate",
        returnPosition: { x: 0.0, y: 0.0 },
        reason: "manual_stop",
      };

      await robotService.stopPatrol(robotId, stopData);
      onCommandSent?.("stop_patrol");
      alert("순찰이 중지되었습니다.");
    } catch (error) {
      console.error("순찰 중지 실패", error);
      alert("순찰 중지에 실패했습니다.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="robot-control">
      <h3>로봇 제어</h3>
      <div className="control-buttons">
        <button
          onClick={handleStartPatrol}
          disabled={loading || currentStatus === "patrolling"}
          className="btn-primary"
        >
          {loading ? "처리 중..." : "순찰 시작"}
        </button>
        <button
          onClick={handleStopPatrol}
          disabled={loading || currentStatus !== "patrolling"}
          className="btn-secondary"
        >
          {loading ? "처리 중..." : "순찰 중지"}
        </button>
      </div>

      {/* 웨이 포인트 설정 UI도 추가 가능 */}
    </div>
  );
};

export default RobotControl;
