export interface User {
  userId: number;
  username: string;
  name: string;
  role: "ADMIN" | "OPERATOR";
  lastLoginAt: string | null;  // null일 수 있음
  isDeleted?: boolean;         // 선택적 필드
  createdAt: string;
  updatedAt?: string;          // 선택적 필드
}

export interface UserListResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data: {
    content: User[];
    page: number;              // 백엔드 실제 구조에 맞춤
    size: number;
    totalElements: number;
    totalPages: number;
    first: boolean;
    last: boolean;
  };
}

export interface CreateUserRequest {
  username: string;
  password: string;
  name: string;
  role: "ADMIN" | "OPERATOR";
}

export interface UpdateUserRequest {
  name: string;
  password: string;
}
