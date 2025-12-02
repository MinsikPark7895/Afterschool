import React from "react";
// import { Navigate } from "react-router-dom"; // 개발 중이므로 주석 처리
import Header from "../components/Header/Header";
import "./MainPage.css";
import { useAuth } from "../hooks/useAuth";
import MenuCard from "../components/Main/MenuCard";

export default function MainPage() {
  const { user, loading, isAdmin } = useAuth(); // user 정보도 사용

  // 로딩 중
  if (loading) {
    return (
      <div className="mainContainer">
        <Header />
        <main className="mainContent">
          <div className="loading">로딩 중...</div>
        </main>
      </div>
    );
  }

  // // 로그인하지 않은 경우 로그인 페이지로 리다이렉트
  // if (!user) {
  //   return <Navigate to="/" />;
  // }

  return (
    <div className="mainContainer">
      <Header />
      <main className="mainContent">
        <div className="welcome-section">
          <h1>AI Based Security Service</h1>
          {user && (
            <p>안녕하세요, {user.name}님! ({user.role === 'ADMIN' ? '관리자' : '운영자'})</p>
          )}
        </div>

        <div className="menu-grid">
          {/* 순찰 관리 - 모든 사용자들에게 표시 */}
          <MenuCard
            title='순찰 관리'
            description='로봇 순찰 상태를 확인하고 관리합니다'
            icon='🤖'
            route='/patrol'
            color='#4A90E2'
          />

          {/* 이벤트 관리 - 모든 사용자들에게 표시 */}
          <MenuCard
            title='이벤트 관리'
            description='침입자 탐지 이벤트를 조회하고 관리합니다'
            icon='🚨'
            route='/events'
            color='#FF6B6B'
          />

          {/* 사용자 관리 - ADMIN만 표시 */}
          {isAdmin && (
            <MenuCard
              title="사용자 관리"
              description="시스템 사용자를 관리합니다"
              icon='👤'
              route='/admin/users'
              color='#7B68EE'
            />
          )}

          {/* 대시보드 - ADMIN만 표시 */}
          {/* {isAdmin && (
            <MenuCard
              title="관리자 대시보드"
              description="시스템 전체 현황을 확인합니다"
              icon='📊'
              route='/admin/dashboard'
              color='#45B7D1'
            />
          )} */}
          {/* 역할별 추가 정보 표시 */}
          {/* <div className="role-info">
            {isOperator && (
              <div className="role-badge operator">
                <span>운영자 권한</span>
                <p>순찰 관리 기능을 사용할 수 있습니다.</p>
              </div>
            )}
            {isAdmin && (
              <div className="role-badge admin">
                <span>관리자 권한</span>
                <p>모든 시스템 기능을 사용할 수 있습니다.</p>
              </div>
            )}
          </div> */}
        </div>
      </main>
    </div>
  );
}
