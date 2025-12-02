import React from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import { useAuth } from '../../hooks/useAuth';
import { authService } from '../../services/authService';

interface ProtectedRouteProps {
  children: React.ReactNode;
  adminOnly?: boolean;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({ children, adminOnly = false }) => {
  const { user, loading } = useAuth();
  const location = useLocation();

  // 로딩 중일 때는 로딩 표시
  if (loading) {
    return (
      <div style={{ 
        display: 'flex', 
        justifyContent: 'center', 
        alignItems: 'center', 
        height: '100vh',
        background: '#1a1a1a',
        color: 'white'
      }}>
        로딩 중...
      </div>
    );
  }

  // 로그인하지 않은 경우 로그인 페이지로 리다이렉트
  if (!user) {
    return <Navigate to="/" state={{ from: location }} replace />;
  }

  // 관리자 전용 페이지인데 관리자가 아닌 경우
  if (adminOnly && user.role !== 'ADMIN') {
    // 운영자는 순찰 페이지로 리다이렉트
    const defaultPage = authService.getDefaultPageByRole(user.role);
    return <Navigate to={defaultPage} replace />;
  }

  // 접근 가능한 경우 자식 컴포넌트 렌더링
  return <>{children}</>;
};

export default ProtectedRoute;
