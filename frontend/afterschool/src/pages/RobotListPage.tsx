import React, { useState, useEffect, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import Header from "../components/Header/Header";
import RobotCard from "../components/Robot/RobotCard";
import { Robot } from "../types/robot";
import { robotService } from "../services/robotService";
import "./RobotListPage.css";

const RobotListPage: React.FC = () => {
  const navigate = useNavigate();
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");
  const [activeFilter, setActiveFilter] = useState<boolean>(true);

  const fetchRobots = useCallback(async () => {
    setLoading(true);
    setError("");

    try {
      const response = await robotService.getRobots();
      if (response.status === "SUCCESS" && response.data) {
        // 데이터가 배열인지 확인하고, 각 로봇 객체에 기본값 설정
        const robotsData = Array.isArray(response.data) ? response.data : [];
        const safeRobots = robotsData.map(robot => ({
          ...robot,
          currentStatus: robot.currentStatus || {
            state: 'unknown',
            position: { x: 0, y: 0, z: 0 },
            batteryLevel: 0,
            lastUpdated: new Date().toISOString()
          }
        }));
        setRobots(safeRobots);
      } else {
        setError(response.message || "로봇 목록을 불러오는데 실패했습니다.");
      }
    } catch (error) {
      console.error("Failed to fetch robots:", error);
      setError("로봇 목록을 불러오는데 실패했습니다. 서버 연결을 확인해주세요.");
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchRobots();
  }, [fetchRobots]);

  const handleRobotClick = (robotId: string) => {
    navigate(`/robots/${robotId}`);
  };

  const handleFilterChange = (isActive: boolean) => {
    setActiveFilter(isActive);
  };

  return (
    <div className="robot-list-page">
      <Header />
      <main className="robot-list-content">
        <div className="page-header">
          <h1 className="page-title">로봇 전체 목록</h1>
          <div className="filter-controls">
            <button
              className={`filter-btn ${activeFilter ? "active" : ""}`}
              onClick={() => handleFilterChange(true)}
            >
              활성 로봇
            </button>
            <button
              className={`filter-btn ${!activeFilter ? "active" : ""}`}
              onClick={() => handleFilterChange(false)}
            >
              비활성 로봇
            </button>
          </div>
        </div>

        {loading && <div className="loading">로딩 중...</div>}
        {error && <div className="error">{error}</div>}

        {!loading && !error && (
          <div className="robot-grid">
            {robots.length > 0 ? (
              robots.map((robot) => (
                <RobotCard
                  key={robot.id}
                  robot={robot}
                  onClick={handleRobotClick}
                />
              ))
            ) : (
              <div className="no-robots">로봇이 없습니다.</div>
            )}
          </div>
        )}
      </main>
    </div>
  );
};

export default RobotListPage;
