import React, { useState, useRef } from "react";
import { User } from "../../types/admin";
import "./EditUserModal.css";

interface EditUserModalProps {
  user: User;
  onClose: () => void;
  onUpdate: (userId: number, userData: { name: string; password: string }) => void;
}

export default function EditUserModal({ user, onClose, onUpdate }: EditUserModalProps) {
  const [formData, setFormData] = useState({
    name: user.name,
    password: ""
  });
  
  // 드래그 시작 위치를 추적하기 위한 ref
  const dragStartRef = useRef<{ x: number; y: number } | null>(null);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (formData.password.trim() === "") {
      alert("비밀번호를 입력해주세요.");
      return;
    }
    onUpdate(user.userId, formData);
  };

  const handleInputChange = (field: keyof typeof formData, value: string) => {
    setFormData(prev => ({
      ...prev,
      [field]: value
    }));
  };

  // 마우스 다운 이벤트 - 드래그 시작 위치 저장
  const handleMouseDown = (e: React.MouseEvent) => {
    dragStartRef.current = { x: e.clientX, y: e.clientY };
  };

  // 오버레이 클릭 이벤트 - 드래그가 아닌 경우만 모달 닫기
  const handleOverlayClick = (e: React.MouseEvent) => {
    // 클릭한 대상이 오버레이가 아니면 무시
    if (e.target !== e.currentTarget) {
      return;
    }

    // 드래그 시작 위치가 없으면 일반 클릭으로 처리
    if (!dragStartRef.current) {
      onClose();
      return;
    }

    // 드래그 거리 계산
    const dragDistance = Math.sqrt(
      Math.pow(e.clientX - dragStartRef.current.x, 2) + 
      Math.pow(e.clientY - dragStartRef.current.y, 2)
    );

    // 드래그 거리가 5픽셀 이하면 클릭으로 간주
    if (dragDistance <= 5) {
      onClose();
    }

    // 드래그 시작 위치 초기화
    dragStartRef.current = null;
  };

  return (
    <div 
      className="modal-overlay" 
      onMouseDown={handleMouseDown}
      onClick={handleOverlayClick}
    >
      <div className="modal-content" onClick={(e) => e.stopPropagation()}>
        <h2>사용자 수정</h2>
        <div className="user-info">
          <p><strong>사용자명:</strong> {user.username}</p>
          <p><strong>역할:</strong> {user.role}</p>
        </div>
        
        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="name">이름</label>
            <input
              id="name"
              type="text"
              placeholder="이름을 입력하세요"
              value={formData.name}
              onChange={(e) => handleInputChange("name", e.target.value)}
              required
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="password">새 비밀번호</label>
            <input
              id="password"
              type="password"
              placeholder="새 비밀번호를 입력하세요"
              value={formData.password}
              onChange={(e) => handleInputChange("password", e.target.value)}
              required
            />
          </div>
          
          <div className="modal-buttons">
            <button type="submit">수정</button>
            <button type="button" onClick={onClose}>취소</button>
          </div>
        </form>
      </div>
    </div>
  );
}
