import React from "react";
import { Robot } from "../../types/robot";
import RobotStatusBadge from "./RobotStatusBadge";
import "./RobotCard.css";

interface RobotCardProps {
  robot: Robot;
  onClick: (robotId: string) => void;
}

const RobotCard: React.FC<RobotCardProps> = ({ robot, onClick }) => {
  return (
    <div className="robot-card" onClick={() => onClick(robot.robotId)}>
      <div className="robot-image-container">
        <img src="/assets/robot.png" alt="없음" />
      </div>
      <div className="robot-info">
        <h3 className="robot-name">1. {robot.robotName}</h3>
        <p className="robot-type">2. {robot.robotType}</p>
        <p className="robot-zone">구역: {robot.assignedZone}</p>
        <RobotStatusBadge
          state={robot.currentStatus?.state || 'unknown'}
          batteryLevel={robot.currentStatus?.batteryLevel || 0}
          isActive={robot.isActive}
        />
      </div>
    </div>
  );
};

export default RobotCard;
