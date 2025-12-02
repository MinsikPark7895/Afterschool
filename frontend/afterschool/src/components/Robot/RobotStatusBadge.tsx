import React from "react";

interface RobotStatusBadgeProps {
  state: string;
  batteryLevel: number;
  isActive: boolean;
}

const RobotStatusBadge: React.FC<RobotStatusBadgeProps> = ({
  state,
  batteryLevel,
  isActive,
}) => {
  const getStateColor = (state: string) => {
    switch (state.toLowerCase()) {
      case "patrolling":
        return "#4CAF50";
      case "charging":
        return "#2196F3";
      case "idle":
        return "#FF9800";
      case "error":
        return "#F44336";
      default:
        return "#9E9E9E";
    }
  };

  const getBatteryColor = (level: number) => {
    if (level > 60) return "#4CAF50";
    if (level > 30) return "#FF9800";
    return "#F44336";
  };

  return (
    <div className="robot-status-badge">
      <div className="status-item">
        <span
          className="status-dot"
          style={{ backgroundColor: getStateColor(state) }}
        ></span>
        <span className="status-text">{state}</span>
      </div>
      <div className="battery-info">
        <span
          className="battery-level"
          style={{ color: getBatteryColor(batteryLevel) }}
        >
          배터리: {batteryLevel.toFixed(1)}%
        </span>
      </div>
      <div className="active-status">
        <span
          className={`active-indicator ${isActive ? "active" : "inactive"}`}
        >
          {isActive ? "활성" : "비활성"}
        </span>
      </div>
    </div>
  );
};

export default RobotStatusBadge;
