import React from "react";
import { Health } from "../../types/robot";

interface RobotHealthInfoProps {
  health: Health;
}

const RobotHealthInfo: React.FC<RobotHealthInfoProps> = ({ health }) => {
  const getHealthColor = (status: string) => {
    switch (status.toLowerCase()) {
      case "healthy":
      case "active":
        return "#4CAF50";
      case "warning":
        return "#FF9800";
      case "error":
      case "inactive":
        return "#F44336";
      default:
        return "#9E9E9E";
    }
  };

  const getPerformanceColor = (
    value: number,
    type: "cpu" | "memory" | "fps"
  ) => {
    if (type === "fps") {
      return value >= 25 ? "#4CAF50" : "#FF9800";
    }
    if (value < 70) return "#4CAF50";
    if (value < 90) return "#FF9800";
    return "#F44336";
  };

  return (
    <div className="robot-health-info">
      <div className="health-section">
        <h4>전체 상태</h4>
        <span
          className="health-status"
          style={{ color: getHealthColor(health.overallStatus) }}
        >
          {health.overallStatus}
        </span>
      </div>

      <div className="health-section">
        <h4>하드웨어 상태</h4>
        <div className="hardware-grid">
          {Object.entries(health.hardware).map(([key, value]) => (
            <div key={key} className="hardware-item">
              <span className="hardware-name">{key.toUpperCase()}</span>
              <span
                className="hardware-status"
                style={{ color: getHealthColor(value) }}
              >
                {value}
              </span>
            </div>
          ))}
        </div>
      </div>

      <div className="health-section">
        <h4>성능 지표</h4>
        <div className="performance-grid">
          <div className="performance-item">
            <span className="performance-name">CPU 사용률</span>
            <span
              className="performance-value"
              style={{
                color: getPerformanceColor(health.performance.cpuUsage, "cpu"),
              }}
            >
              {health.performance.cpuUsage.toFixed(1)}%
            </span>
          </div>
          <div className="performance-item">
            <span className="performance-name">메모리 사용률</span>
            <span
              className="performance-value"
              style={{
                color: getPerformanceColor(
                  health.performance.memoryUsage,
                  "memory"
                ),
              }}
            >
              {health.performance.memoryUsage.toFixed(1)}%
            </span>
          </div>
          <div className="performance-item">
            <span className="performance-name">FPS</span>
            <span
              className="performance-value"
              style={{
                color: getPerformanceColor(health.performance.fps, "fps"),
              }}
            >
              {health.performance.fps.toFixed(1)}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotHealthInfo;
