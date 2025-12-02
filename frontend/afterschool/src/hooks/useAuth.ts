import { useState, useEffect } from "react";

interface User {
  userId: number;
  username: string;
  name: string; // API 명세서에 따라 필수 필드
  role: "ADMIN" | "OPERATOR";
  lastLoginAt: string;
  createdAt?: string | null; // 생성일 (선택적)
  updatedAt?: string | null; // 수정일 (선택적)
}

export const useAuth = () => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    try {
      const storedUser = localStorage.getItem("user");
      if (storedUser) {
        setUser(JSON.parse(storedUser));
      }
    } catch (error) {
      console.error("Failed to parse user data: ", error);
      localStorage.removeItem("user");
    } finally {
      setLoading(false);
    }
  }, []);

  const logout = () => {
    localStorage.removeItem("accessToken");
    localStorage.removeItem("refreshToken");
    localStorage.removeItem("user");
    setUser(null);
  };

  return {
    user,
    loading,
    logout,
    isAdmin: user?.role === "ADMIN",
    isOperator: user?.role === "OPERATOR",
  };
};
