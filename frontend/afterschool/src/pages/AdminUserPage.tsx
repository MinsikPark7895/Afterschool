import React, { useState, useEffect, useCallback } from "react";
import Header from "../components/Header/Header";
import CreateUserModal from "../components/Admin/CreateUserModal";
import EditUserModal from "../components/Admin/EditUserModal";
import UserTable from "../components/Admin/UserTable";
import { User, CreateUserRequest } from "../types/admin";
import { useAuth } from "../hooks/useAuth";
import "./AdminUserPage.css";

export default function AdminUserPage() {
  const { user, loading: authLoading, isAdmin } = useAuth();
  
  const [users, setUsers] = useState<User[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");
  const [roleFilter, setRoleFilter] = useState<"ADMIN" | "OPERATOR" | "">("");
  const [showCreateModal, setShowCreateModal] = useState(false);
  const [editingUser, setEditingUser] = useState<User | null>(null);

  // // 권한 체크
  // useEffect(() => {
  //   if (!authLoading && (!user || !isAdmin)) {
  //     navigate('/main');
  //   }
  // }, [user, isAdmin, authLoading, navigate]);

  // 사용자 목록 조회
  const fetchUsers = async (
    page = 0,
    size = 20,
    role = "",
    sort = "createdAt,desc"
  ) => {
    const token = localStorage.getItem("accessToken");
    
    if (!token) {
      throw new Error("인증 토큰이 없습니다. 다시 로그인해주세요.");
    }

    // // 백엔드에서 처리하는 파라미터만 전송
    // const params = new URLSearchParams({
    //   page: page.toString(),
    //   size: size.toString(),
    //   sort,
    // });

    // role과 isDeleted 파라미터는 백엔드에서 처리하지 않으므로 제거

    // 요청 정보 로깅
    console.log("=== API 요청 정보 ===");
    console.log("URL:", `/api/admin/users`);
    console.log("Method:", "GET");
    console.log("Headers:", {
      Authorization: `Bearer ${token}`,
      "Content-Type": "application/json",
    });
    console.log("Query Parameters: 없음 (빈 요청)");

    const response = await fetch(`/api/admin/users`, {
      method: "GET",
      headers: {
        Authorization: `Bearer ${token}`,
        "Content-Type": "application/json",
      },
    });

    // 응답 상태 로깅
    console.log("=== API 응답 정보 ===");
    console.log("Status:", response.status);
    console.log("Status Text:", response.statusText);
    console.log("Headers:", Object.fromEntries(response.headers.entries()));

    if (response.status === 401) {
      // 토큰 만료 또는 권한 없음
      localStorage.removeItem("accessToken");
      localStorage.removeItem("refreshToken");
      localStorage.removeItem("user");
      window.location.href = '/';
      throw new Error("인증이 만료되었습니다. 다시 로그인해주세요.");
    }

    if (response.status === 403) {
      // 권한 부족 - 백엔드에서 권한 체크 실패
      throw new Error("관리자 권한이 필요합니다. 관리자에게 문의하세요.");
    }

    // 응답 본문을 먼저 읽기 (에러 상태라도)
    const responseText = await response.text();
    console.log("Response Body (Raw Text):", responseText);
    
    let responseJson;
    try {
      responseJson = JSON.parse(responseText);
      console.log("Response Body (Parsed JSON):", responseJson);
    } catch (parseError) {
      console.error("JSON 파싱 실패:", parseError);
      console.log("Raw response:", responseText);
    }

    // 에러 상태 처리
    if (response.status === 500) {
      console.error("=== 500 서버 내부 오류 ===");
      console.error("서버 응답:", responseJson || responseText);
      throw new Error("서버에서 오류가 발생했습니다. 잠시 후 다시 시도해주세요.");
    }

    if (!response.ok) {
      console.error(`HTTP 오류 발생: ${response.status} - ${response.statusText}`);
      console.error("오류 응답 내용:", responseJson || responseText);
      throw new Error(`서버 오류가 발생했습니다. (상태코드: ${response.status})`);
    }

    return responseJson;
  };

  // 사용자 생성
  const createUser = async (userData: CreateUserRequest) => {
    const token = localStorage.getItem("accessToken");

    const response = await fetch("/api/admin/create-user", {
      method: "POST",
      headers: {
        Authorization: `Bearer ${token}`,
        "Content-Type": "application/json",
      },
      body: JSON.stringify(userData),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  };

  // 사용자 수정
  const updateUser = async (
    userId: number,
    userData: { name: string; password: string }
  ) => {
    const token = localStorage.getItem("accessToken");

    const response = await fetch(`/api/admin/update-user/${userId}`, {
      method: "PUT",
      headers: {
        Authorization: `Bearer ${token}`,
        "Content-Type": "application/json",
      },
      body: JSON.stringify(userData),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  };

  // 사용자 삭제
  const deleteUser = async (userId: number) => {
    const token = localStorage.getItem("accessToken");

    const response = await fetch(`/api/admin/delete-user/${userId}`, {
      method: "DELETE",
      headers: {
        Authorization: `Bearer ${token}`,
        "Content-Type": "application/json",
      },
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return response.json();
  };

  const handleLoadUsers = useCallback(async () => {
    setLoading(true);
    try {
      console.log("=== 사용자 목록 로딩 시작 ===");
      // role 파라미터는 제거하고 기본 파라미터만 전송
      const result = await fetchUsers();
      
      console.log("=== fetchUsers 결과 ===");
      console.log("Full result:", result);
      
      if (result && result.data && Array.isArray(result.data.content)) {
        console.log("사용자 데이터 수신 성공:");
        console.log("- 전체 사용자 수:", result.data.content.length);
        console.log("- 사용자 목록:", result.data.content);
        
        // 프론트엔드에서 role 필터링 처리
        let filteredUsers = result.data.content;
        if (roleFilter) {
          filteredUsers = result.data.content.filter((user: User) => user.role === roleFilter);
          console.log(`- ${roleFilter} 필터 적용 후:`, filteredUsers.length, "명");
        }
        setUsers(filteredUsers);
      } else {
        console.error("잘못된 데이터 형식:");
        console.error("- result:", result);
        console.error("- result.data:", result?.data);
        console.error("- result.data.content type:", typeof result?.data?.content);
        setUsers([]);
        setError("사용자 데이터 형식이 올바르지 않습니다.");
      }
    } catch (error) {
      console.error("=== 사용자 목록 로딩 실패 ===");
      console.error("Error:", error);
      console.error("Error message:", error instanceof Error ? error.message : "Unknown error");
      setUsers([]);
      setError("사용자 목록을 불러오는데 실패했습니다.");
    } finally {
      setLoading(false);
      console.log("=== 사용자 목록 로딩 완료 ===");
    }
  }, [roleFilter]); // currentPage, pageSize 제거 (더 이상 사용하지 않음)

  const handleCreateUser = async (userData: CreateUserRequest) => {
    try {
      const result = await createUser(userData);
      if (result.status === "SUCCESS") {
        handleLoadUsers(); //  목록 새로 고침
        setError(""); // 성공 시 에러 메시지 초기화
      } else {
        setError(result.message || "사용자 생성에 실패했습니다.");
      }
    } catch (error) {
      setError("사용자 생성에 실패했습니다.");
      console.error("Create user error:", error);
    } finally {
      // 성공/실패와 관계없이 모달 닫기
      setShowCreateModal(false);
    }
  };

  const handleEditUser = async (
    userId: number,
    userData: { name: string; password: string }
  ) => {
    try {
      const result = await updateUser(userId, userData);
      if (result.status === "SUCCESS") {
        handleLoadUsers(); // 목록 새로 고침
        setError(""); // 성공 시 에러 메시지 초기화
      } else {
        setError(result.message || "사용자 수정에 실패했습니다.");
      }
    } catch (error) {
      setError("사용자 수정에 실패했습니다.");
      console.error("Update user error:", error);
    } finally {
      // 성공/실패와 관계없이 모달 닫기
      setEditingUser(null);
    }
  };

  const handleDeleteUser = async (userId: number) => {
    if (window.confirm("정말 이 사용자를 삭제하시겠습니까?")) {
      try {
        const result = await deleteUser(userId);
        if (result.status === "SUCCESS") {
          handleLoadUsers(); // 목록 새로 고침
          setError(""); // 성공 시 에러 메시지 초기화
        } else {
          setError(result.message || "사용자 삭제에 실패했습니다.");
        }
      } catch (error) {
        setError("사용자 삭제에 실패했습니다.");
        console.error("Delete user error:", error);
      }
    }
  };

  useEffect(() => {
    // if (isAdmin && user) {
    handleLoadUsers();
    // }
  }, [handleLoadUsers, isAdmin, user]);

  // 인증 로딩 중
  if (authLoading) {
    return (
      <div className="admin-user-page">
        <Header />
        <div className="admin-content">
          <div style={{ textAlign: 'center', padding: '2rem' }}>
            로딩 중...
          </div>
        </div>
      </div>
    );
  }

  // // 권한 없음
  // if (!user || !isAdmin) {
  //   return (
  //     <div className="admin-user-page">
  //       <Header />
  //       <div className="admin-content">
  //         <div style={{ textAlign: 'center', padding: '2rem' }}>
  //           관리자 권한이 필요합니다.
  //         </div>
  //       </div>
  //     </div>
  //   );
  // }

  return (
    <div className="admin-user-page">
      <Header />
      
      <div className="admin-content">
        <div className="admin-user-header">
          <h1>관리자 - 사용자 정보</h1>
        </div>

        {error && (
          <div className="error-message" style={{ 
            color: 'red', 
            marginBottom: '1rem',
            padding: '1rem',
            backgroundColor: '#fee',
            border: '1px solid #fcc',
            borderRadius: '4px'
          }}>
            <div>{error}</div>
          </div>
        )}

      {/* 필터 및 검색 */}
      <div className="filters">
        <select
          value={roleFilter}
          onChange={(event) => setRoleFilter(event.target.value as "ADMIN" | "OPERATOR" | "")}
        >
          <option value="">모든 권한</option>
          <option value="ADMIN">관리자</option>
          <option value="OPERATOR">운영자</option>
        </select>
        <button onClick={() => setShowCreateModal(true)}>사용자 추가</button>
      </div>

      {/* 사용자 테이블 */}
      <div className="user-table-container">
        <UserTable
          users={users}
          loading={loading}
          onEdit={setEditingUser}
          onDelete={handleDeleteUser}
          isAdmin={isAdmin}
        />
      </div>

      {/* 페이지네이션 - 빈 JSON 요청 사용으로 비활성화 */}
      <div className="pagination" style={{ opacity: 0.5, pointerEvents: 'none' }}>
        <button disabled>
          이전
        </button>
        <span>
          모든 데이터 표시 중
        </span>
        <button disabled>
          다음
        </button>
      </div>

      {/* 모달들 */}
      {showCreateModal && (
        <CreateUserModal
          onClose={() => setShowCreateModal(false)}
          onCreate={handleCreateUser}
        />
      )}
      {editingUser && (
        <EditUserModal
          user={editingUser}
          onClose={() => setEditingUser(null)}
          onUpdate={handleEditUser}
        />
      )}
      </div>
    </div>
  );
}
