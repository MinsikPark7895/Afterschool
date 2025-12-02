// 이 코드 아직 이해 못함

import React, { useState, useEffect } from "react";
import { useParams, useNavigate } from "react-router-dom";
import Header from "../components/Header/Header";
import RobotHealthInfo from "../components/Robot/RobotHealthInfo";
import RobotControl from "../components/Robot/RobotControl";
import { RobotDetail } from "../types/robot";
import { robotService } from "../services/robotService";
import "./RobotDetailPage.css";

const RobotDetailPage: React.FC = () => {
  const { robotId } = useParams<{ robotId: string }>();
  const navigate = useNavigate();
  const [robot, setRobot] = useState<RobotDetail | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");

  useEffect(() => {
    if (robotId) {
      fetchRobotDetail(robotId);
    }
  }, [robotId]);

  const fetchRobotDetail = async (robotId: string) => {
    setLoading(true);
    setError("");
    try {
      const response = await robotService.getRobotDetail(robotId);
      if (response.status === "SUCCESS") {
        setRobot(response.data);
      } else {
        setError(response.message || "로봇 정보를 불러오는데 실패했습니다.");
      }
    } catch (error: any) {
      console.error("Failed to fetch robot detail:", error);
      if (error.message === "ROBOT_NOT_FOUND") {
        setError("로봇을 찾을 수 없습니다.");
      } else {
        setError("로봇 정보를 불러오는데 실패했습니다.");
      }
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <div className="robot-detail-page">
        <Header />
        <div className="loading">로딩 중...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="robot-detail-page">
        <Header />
        <div className="error-container">
          <div className="error">{error}</div>
          <button className="back-button" onClick={() => navigate("/robots")}>
            목록으로 돌아가기
          </button>
        </div>
      </div>
    );
  }

  if (!robot) {
    return (
      <div className="robot-detail-page">
        <Header />
        <div className="error">로봇 정보를 찾을 수 없습니다.</div>
      </div>
    );
  }

  return (
    <div className="robot-detail-page">
      <Header />
      <main className="robot-detail-content">
        <h1 className="page-title">로봇 상세</h1>

        <div className="robot-detail-card">
          <div className="robot-image-section">
            <div className="robot-image-container-large">
              <img
                src="/assets/images/robot-placeholder.png"
                alt={robot.robotName}
                className="robot-image-large"
              />
            </div>
          </div>

          <div className="robot-info-section">
            <div className="robot-info-item">
              <span className="info-number">1.</span>
              <span className="info-text">{robot.robotName}</span>
            </div>
            <div className="robot-info-item">
              <span className="info-number">2.</span>
              <span className="info-text">{robot.detailedStatus.state}</span>
            </div>
            <div className="robot-info-item">
              <span className="info-number">3.</span>
              <span className="info-text">
                배터리: {robot.detailedStatus.battery.level.toFixed(1)}% (
                {robot.detailedStatus.battery.chargingStatus})
              </span>
            </div>
            <div className="robot-info-item">
              <span className="info-number">4.</span>
              <span className="info-text">
                위치: ({robot.detailedStatus.position.x.toFixed(1)},{" "}
                {robot.detailedStatus.position.y.toFixed(1)})
              </span>
            </div>
          </div>
        </div>

        <div className="robot-health-section">
          <h2>로봇 상태 정보</h2>
          <RobotHealthInfo health={robot.detailedStatus.health} />
        </div>

        <div className="robot-control-section">
          <RobotControl 
            robotId={robot.robotId}
            currentStatus={robot.detailedStatus.state}
            onCommandSent={(commandType) => {
              console.log(`Command sent: ${commandType}`);
              // 명령 전송 후 로봇 상태 다시 조회
              if (robotId) {
                fetchRobotDetail(robotId);
              }
            }}
          />
        </div>

        <div className="robot-additional-info">
          <div className="info-grid">
            <div className="info-card">
              <h3>기본 정보</h3>
              <p>
                <strong>로봇 ID:</strong> {robot.robotId}
              </p>
              <p>
                <strong>타입:</strong> {robot.robotType}
              </p>
              <p>
                <strong>할당 구역:</strong> {robot.assignedZone}
              </p>
              <p>
                <strong>활성 상태:</strong> {robot.isActive ? "활성" : "비활성"}
              </p>
            </div>
            <div className="info-card">
              <h3>최근 업데이트</h3>
              <p>
                <strong>상태 업데이트:</strong>{" "}
                {new Date(robot.detailedStatus.lastUpdated).toLocaleString()}
              </p>
              <p>
                <strong>정보 수정:</strong>{" "}
                {new Date(robot.updatedAt).toLocaleString()}
              </p>
              <p>
                <strong>등록일:</strong>{" "}
                {new Date(robot.createdAt).toLocaleString()}
              </p>
            </div>
          </div>
        </div>

        <button className="back-button" onClick={() => navigate("/robots")}>
          목록으로 돌아가기
        </button>
      </main>
    </div>
  );
};

export default RobotDetailPage;
