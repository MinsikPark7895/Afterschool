import React, { useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";
import { authService } from "../../services/authService";
import "./ProfileModal.css";

interface UserProfileData {
  userId: number;
  username: string;
  name: string;
  role: "ADMIN" | "OPERATOR";
  createdAt: string;
  lastLoginAt: string;
}

interface ProfileModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const ProfileModal: React.FC<ProfileModalProps> = ({
  isOpen,
  onClose,
}) => {
  const navigate = useNavigate();
  const modalRef = useRef<HTMLDivElement>(null);
  const [userProfile, setUserProfile] = useState<UserProfileData | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");

  // 프로필 정보 가져오기
  useEffect(() => {
    if (isOpen) {
      fetchUserProfile();
    }
  }, [isOpen]);

  const fetchUserProfile = async () => {
    setLoading(true);
    setError("");
    try {
      const response = await authService.getMyProfile();
      if (response.status === "SUCCESS" && response.data) {
        setUserProfile(response.data);
      } else {
        setError(response.message || "프로필 정보를 가져올 수 없습니다.");
      }
    } catch (err) {
      setError("프로필 정보를 가져오는 중 오류가 발생했습니다.");
      console.error("Profile fetch error:", err);
    } finally {
      setLoading(false);
    }
  };

  // 배경 스크롤 방지
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = "hidden";
    }

    return () => {
      document.body.style.overflow = "unset";
    };
  }, [isOpen]);

  // 배경 클릭 시 모달 닫기
  const handleOverlayClick = (event: React.MouseEvent<HTMLDivElement>) => {
    if (event.target === event.currentTarget) {
      onClose();
    }
  };

  // 포커스 트랩
  useEffect(() => {
    if (isOpen && modalRef.current) {
      const focusableElements = modalRef.current.querySelectorAll(
        'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
      );
      const firstElement = focusableElements[0] as HTMLElement;
      const lastElement = focusableElements[
        focusableElements.length - 1
      ] as HTMLElement;

      const handleTabKey = (event: KeyboardEvent) => {
        if (event.key === "Tab") {
          if (event.shiftKey) {
            if (document.activeElement === firstElement) {
              lastElement.focus();
              event.preventDefault();
            }
          } else {
            if (document.activeElement === lastElement) {
              firstElement.focus();
              event.preventDefault();
            }
          }
        }
      };

      firstElement?.focus();
      document.addEventListener("keydown", handleTabKey);

      return () => {
        document.removeEventListener("keydown", handleTabKey);
      };
    }
  }, [isOpen]);

  // 날짜 형식 변환 함수
  const formatDate = (dateString: string) => {
    try {
      const date = new Date(dateString);
      return date.toLocaleDateString('ko-KR', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit'
      });
    } catch {
      return "정보 없음";
    }
  };

  // // 역할별 배지 렌더링 함수
  // const renderRoleBadge = (role: "ADMIN" | "OPERATOR") => {
  //   if (role === "ADMIN") {
  //     return (
  //       <div className="roleBadge adminBadge">
  //         ADMIN
  //       </div>
  //     );
  //   } else if (role === "OPERATOR") {
  //     return (
  //       <div className="roleBadge operatorBadge">
  //         OPERATOR
  //       </div>
  //     );
  //   }
  //   return null;
  // };

  if (!isOpen) return null;

  // 로그아웃
  const handleLogout = async () => {
    try {
      const accessToken = localStorage.getItem("accessToken");

      if (!accessToken) {
        // 토큰이 없으면 이미 로그아웃된 상태
        clearUserData();
        navigate("/");
        return;
      }

      // authService를 사용한 로그아웃
      const result = await authService.logout();
      
      if (result.status === "SUCCESS") {
        console.log("로그아웃 성공:", result.message);
      } else {
        console.warn("로그아웃 응답 경고:", result.message);
      }

      // 로컬 스토리지에서 사용자 데이터 삭제
      clearUserData();
      navigate("/");
    } catch (error) {
      // 네트워크 에러가 있어도 클라이언트에서는 로그아웃 처리
      console.error("로그아웃 중 오류:", error);
      // 서버 에러가 있어도 클라이언트에서는 로그아웃 처리를 완료
      clearUserData();
      navigate("/");
    }
  };

  const clearUserData = () => {
    localStorage.removeItem("accessToken");
    localStorage.removeItem("refreshToken");
    localStorage.removeItem("user");
  };

  return (
    <div className="overlay" onClick={handleOverlayClick}>
      <div className="modalContainer" ref={modalRef}>
        <div className="modalContent">
          {/* 모달 헤더 */}
          <div className="modalHeader">
            <h2>프로필</h2>
            <button
              className="closeButton"
              onClick={onClose}
              aria-label="모달 닫기"
            >
              X
            </button>
          </div>

          {/* 프로필 정보 */}
          <div className="profileInfo">
            {loading ? (
              <div className="loading">프로필 정보를 불러오는 중...</div>
            ) : error ? (
              <div className="error">오류: {error}</div>
            ) : userProfile ? (
              <>
                <div className="avatarSection">
                  <div className="avatar">
                    <div className="defaultAvatar">
                      {userProfile.name || "U"}
                    </div>
                  </div>
                </div>

                <div className="userDetails">
                  <div className="userField">
                    <span className="fieldLabel">사용자명:</span>
                    <span className="fieldValue">{userProfile.username}</span>
                  </div>
                  <div className="userField">
                    <span className="fieldLabel">역할:</span>
                    <span className="fieldValue">{userProfile.role}</span>
                  </div>
                  <div className="userField">
                    <span className="fieldLabel">가입일:</span>
                    <span className="fieldValue">{formatDate(userProfile.createdAt)}</span>
                  </div>
                  <div className="userField">
                    <span className="fieldLabel">마지막 로그인:</span>
                    <span className="fieldValue">{formatDate(userProfile.lastLoginAt)}</span>
                  </div>
                </div>
              </>
            ) : (
              <div className="no-data">프로필 정보가 없습니다.</div>
            )}
          </div>
          {/* 액션 버튼 */}
          <div className="actionButtons">
            <button className="logoutButton" onClick={handleLogout}>
              로그아웃
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ProfileModal;
