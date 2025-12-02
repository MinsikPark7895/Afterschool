import React, { useState } from "react";
import { EmergencyStopRequest } from "../../types/robot";
import { robotService } from "../../services/robotService";

const EmergencyStop: React.FC = () => {
  const [loading, setLoading] = useState(false);

  const handleEmergencyStop = async () => {
    const confirmed = window.confirm("모든 로봇을 긴급 정지 하시겠습니까?");
    if (!confirmed) return;

    setLoading(true);

    try {
      const stopData: EmergencyStopRequest = {
        stopAllRobots: true,
        stopMode: "immediate",
        disableMotors: true,
        reason: "manual_intervention",
        description: "관리자에 의한 수동 긴급정지",
      };

      await robotService.emergencyStop(stopData);
      alert("모든 로봇이 긴급정지되었습니다.");
    } catch (error) {
      console.error("긴급정지 실패", error);
      const errorMessage = error instanceof Error ? error.message : "긴급정지에 실패했습니다.";
      alert(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  return (
    <button
      onClick={handleEmergencyStop}
      disabled={loading}
      className="btn-emergency"
      style={{ backgroundColor: "#ef4444", color: "white" }}
    >
      {loading ? "처리 중..." : "전체 긴급 정지"}
    </button>
  );
};

export default EmergencyStop;