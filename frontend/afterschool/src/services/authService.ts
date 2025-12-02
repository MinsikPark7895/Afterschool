const BASE_URL = process.env.REACT_APP_API_URL || '/api';

// 로그인 응답 타입 (백엔드 실제 응답 구조에 맞춤)
export interface LoginResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: {
    userId: number;
    username: string;
    role: "ADMIN" | "OPERATOR";
    accessToken: string;
    refreshToken: string;
  };
}

// 프로필 조회 응답 타입
export interface ProfileResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: {
    userId: number;
    username: string;
    name: string;
    role: "ADMIN" | "OPERATOR";
    createdAt: string;
    lastLoginAt: string;
  };
}

// 토큰 갱신 응답 타입 (백엔드와 동일한 구조)
export interface RefreshTokenResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: {
    userId: number;
    username: string;
    role: "ADMIN" | "OPERATOR";
    accessToken: string;
    refreshToken: string;
  };
}

export const authService = {
  // 로그인
  async login(username: string, password: string): Promise<LoginResponse> {
    const response = await fetch(`${BASE_URL}/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ username, password }),
    });

    return await response.json();
  },

  // 역할별 기본 페이지 반환
  getDefaultPageByRole(role: "ADMIN" | "OPERATOR"): string {
    switch (role) {
      case 'ADMIN':
        return '/main';
      case 'OPERATOR':
        return '/patrol';
      default:
        return '/patrol';
    }
  },

  // 로그아웃
  async logout(): Promise<{ status: string; message: string }> {
    const accessToken = localStorage.getItem('accessToken');
    const refreshToken = localStorage.getItem('refreshToken');
    
    const response = await fetch(`${BASE_URL}/auth/logout`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${accessToken}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ refreshToken }),
    });

    return await response.json();
  },

  // 토큰 갱신
  async refreshToken(): Promise<RefreshTokenResponse> {
    const refreshToken = localStorage.getItem('refreshToken');
    
    const response = await fetch(`${BASE_URL}/auth/refresh`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ refreshToken }),
    });

    return await response.json();
  },

  // 내 프로필 조회
  async getMyProfile(): Promise<ProfileResponse> {
    const accessToken = localStorage.getItem('accessToken');
    
    const response = await fetch(`${BASE_URL}/users/me`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${accessToken}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 토큰 유효성 검사 및 자동 갱신
  async validateAndRefreshToken(): Promise<boolean> {
    try {
      const refreshToken = localStorage.getItem('refreshToken');
      if (!refreshToken) {
        return false;
      }

      const response = await this.refreshToken();
      
      if (response.status === 'SUCCESS' && response.data) {
        // 새로운 토큰들 저장
        localStorage.setItem('accessToken', response.data.accessToken);
        localStorage.setItem('refreshToken', response.data.refreshToken);
        
        // 사용자 정보 업데이트
        const userInfo = {
          userId: response.data.userId,
          username: response.data.username,
          name: response.data.username, // name이 없으므로 username으로 대체
          role: response.data.role,
          lastLoginAt: new Date().toISOString() // 현재 시간으로 설정
        };
        localStorage.setItem('user', JSON.stringify(userInfo));
        
        return true;
      }
      
      return false;
    } catch (error) {
      console.error('Token refresh failed:', error);
      return false;
    }
  },

  // 현재 사용자 역할 확인
  getCurrentUserRole(): "ADMIN" | "OPERATOR" | null {
    try {
      const storedUser = localStorage.getItem('user');
      if (storedUser) {
        const user = JSON.parse(storedUser);
        return user.role;
      }
      return null;
    } catch {
      return null;
    }
  },

  // 현재 사용자가 접근 가능한 페이지인지 확인
  canAccessPage(path: string, userRole: "ADMIN" | "OPERATOR"): boolean {
    const adminOnlyPages = ['/admin/users', '/admin/dashboard'];
    
    if (adminOnlyPages.some(page => path.startsWith(page))) {
      return userRole === 'ADMIN';
    }
    
    return true; // 다른 모든 페이지는 접근 가능
  }
};
