import React, { useState, useRef } from "react";
import { CreateUserRequest } from "../../types/admin";
import "./CreateUserModal.css";

interface CreateUserModalProps {
  onClose: () => void;
  onCreate: (userData: CreateUserRequest) => void;
}

export default function CreateUserModal({ onClose, onCreate }: CreateUserModalProps) {
  const [formData, setFormData] = useState<CreateUserRequest>({
    username: "",
    password: "",
    name: "",
    role: "OPERATOR"
  });
  
  // 드래그 시작 위치를 추적하기 위한 ref
  const dragStartRef = useRef<{ x: number; y: number } | null>(null);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onCreate(formData);
  };

  const handleInputChange = (field: keyof CreateUserRequest, value: string) => {
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
        <h2>사용자 추가</h2>
        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="username">사용자명</label>
            <input
              id="username"
              type="text"
              placeholder="사용자명을 입력하세요"
              value={formData.username}
              onChange={(e) => handleInputChange("username", e.target.value)}
              required
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="password">비밀번호</label>
            <input
              id="password"
              type="password"
              placeholder="비밀번호를 입력하세요"
              value={formData.password}
              onChange={(e) => handleInputChange("password", e.target.value)}
              required
            />
          </div>
          
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
            <label htmlFor="role">역할</label>
            <select
              id="role"
              value={formData.role}
              onChange={(e) => handleInputChange("role", e.target.value as "ADMIN" | "OPERATOR")}
            >
              <option value="OPERATOR">운영자</option>
              <option value="ADMIN">관리자</option>
            </select>
          </div>
          
          <div className="modal-buttons">
            <button type="submit">생성</button>
            <button type="button" onClick={onClose}>취소</button>
          </div>
        </form>
      </div>
    </div>
  );
}
