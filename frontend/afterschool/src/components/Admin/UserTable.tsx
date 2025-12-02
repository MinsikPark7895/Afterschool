import React from "react";
import { User } from "../../types/admin";
import "./UserTable.css";

interface UserTableProps {
  users: User[];
  loading: boolean;
  onEdit: (user: User) => void;
  onDelete: (userId: number) => void;
  isAdmin: boolean;   // 관리자 권한 확인시 필요
}

export default function UserTable({
  users,
  loading,
  onEdit,
  onDelete,
  isAdmin = false,
}: UserTableProps) {
  if (loading) {
    return (
      <div className="user-table">
        <div className="loading">로딩 중...</div>
      </div>
    );
  }

  return (
    <div className="user-table">
      <table>
        <thead>
          <tr>
            <th>Number</th>
            <th>이름</th>
            <th>사용자명</th>
            <th>역할</th>
            <th>생성일</th>
            <th>최근 로그인</th>
            {isAdmin && <th>수정</th>}
            {isAdmin && <th>삭제</th>}
          </tr>
        </thead>
        <tbody>
          {users.length === 0 ? (
            <tr>
              <td colSpan={isAdmin ? 8 : 6} style={{ textAlign: "center", padding: "2rem" }}>
                사용자가 없습니다.
              </td>
            </tr>
          ) : (
            users.map((user, index) => (
              <tr key={user.userId}>
                <td>{index + 1}</td>
                <td>{user.username}</td>
                <td>{user.name}</td>
                <td>
                  <span className={`role-badge ${user.role.toLowerCase()}`}>
                    {user.role === "ADMIN" ? "관리자" : "운영자"}
                  </span>
                </td>
                <td>{new Date(user.createdAt).toLocaleDateString()}</td>
                <td>
                  {user.lastLoginAt 
                    ? new Date(user.lastLoginAt).toLocaleDateString()
                    : "로그인 기록 없음"
                  }
                </td>
                {isAdmin && <td>
                  <div className="action-buttons">
                    <button className="edit-btn" onClick={() => onEdit(user)}>
                      수정
                    </button>
                  </div>
                </td>}
                {isAdmin && <td>
                  <div className="action-buttons">
                    <button
                      className="delete-btn"
                      onClick={() => onDelete(user.userId)}
                    >
                      삭제
                    </button>
                  </div>
                </td>}
              </tr>
            ))
          )}
        </tbody>
      </table>
    </div>
  );
}
