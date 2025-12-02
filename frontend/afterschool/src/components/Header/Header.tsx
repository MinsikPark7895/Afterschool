import React, { useState, useEffect } from "react";
import { Link } from "react-router-dom";
import ProfileModal from "../Profile/ProfileModal";
import LogoSection from "./LogoSection";
import ProfileButton from "./ProfileButton";
import MobileMenuButton from "./MobileMenuButton";
// import EmergencyStop from "../Robot/EmergencyStop";
import "./Header.css";
import { useAuth } from "../../hooks/useAuth";
import { authService } from "../../services/authService";

const Header: React.FC = () => {
  const [isProfileModalOpen, setIsProfileModalOpen] = useState(false);
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);
  const { user, isAdmin } = useAuth();

  // 사용자 권한에 따른 로고 링크 결정
  const getLogoHref = () => {
    if (!user) return "/";
    return authService.getDefaultPageByRole(user.role);
  };

  const handleProfileClick = () => {
    setIsProfileModalOpen(true);
  };

  const handleCloseProfileModal = () => {
    setIsProfileModalOpen(false);
  };

  const toggleMobileMenu = () => {
    setIsMobileMenuOpen(!isMobileMenuOpen);
  };

  const closeMobileMenu = () => {
    setIsMobileMenuOpen(false);
  };

  // 화면 크기가 변경될 때 모바일 메뉴 닫기
  useEffect(() => {
    const handleResize = () => {
      if (window.innerWidth > 768) {
        setIsMobileMenuOpen(false);
      }
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  return (
    <>
      <header className="header">
        <div className="header-content">
          {/* 로고 영역 - 사용자 권한에 따른 동적 링크 */}
          <LogoSection to={getLogoHref()} />

          {/* 데스크톱 네비게이션 영역 */}
          <nav className="nav-section desktop-nav">
            <Link to="/patrol" className="nav-link">
              순찰
            </Link>
            <Link to="/events" className="nav-link">
              이벤트
            </Link>
            {/* 관리자만 표시 */}
            {isAdmin && (
              <>
                <Link to="/admin/users" className="nav-link admin-link">
                  사용자 관리
                </Link>
              </>
            )}
            {/* 긴급정지 버튼 - 모든 사용자에게 표시 */}
            {/* <EmergencyStop /> */}
            <ProfileButton onClick={handleProfileClick} />
          </nav>

          {/* 모바일 햄버거 메뉴 버튼 */}
          <MobileMenuButton 
            isOpen={isMobileMenuOpen} 
            onClick={toggleMobileMenu} 
          />
        </div>

        {/* 모바일 드롭다운 메뉴 */}
        {isMobileMenuOpen && (
          <div className="mobile-menu-overlay" onClick={closeMobileMenu}>
            <nav className="mobile-nav" onClick={(e) => e.stopPropagation()}>
              <Link 
                to="/patrol" 
                className="mobile-nav-link"
                onClick={closeMobileMenu}
              >
                순찰
              </Link>
              <Link 
                to="/events" 
                className="mobile-nav-link"
                onClick={closeMobileMenu}
              >
                이벤트
              </Link>
              {/* 관리자만 표시 */}
              {isAdmin && (
                <>
                  <Link 
                    to="/admin/users" 
                    className="mobile-nav-link admin-link"
                    onClick={closeMobileMenu}
                  >
                    사용자 관리
                  </Link>
                </>
              )}
              <div className="mobile-profile-section">
                <ProfileButton onClick={() => {
                  handleProfileClick();
                  closeMobileMenu();
                }} />
              </div>
            </nav>
          </div>
        )}
      </header>

      {/* 프로필 모달 */}
      <ProfileModal
        isOpen={isProfileModalOpen}
        onClose={handleCloseProfileModal}
      />
    </>
  );
};

export default Header;
