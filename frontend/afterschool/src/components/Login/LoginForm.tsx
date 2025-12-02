import { FormEvent, useState } from "react";
import { useNavigate } from "react-router-dom";
import BrandLogo from "../../assets/icons/Logo/BrandLogo";
import { authService } from "../../services/authService";

export default function LoginForm() {
  const navigate = useNavigate();
  const [formData, setFormData] = useState({
    username: "",
    password: ""
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string>("");
  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = event.target;
    setFormData(prev => ({
      ...prev,
      [name]: value,
    }));
    if (error) setError(""); // 에러 메시지 초기화
  };

  const handleSubmit = async (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    setIsLoading(true);
    setError("");

    try {
      const result = await authService.login(formData.username, formData.password);

      if (result.status === "SUCCESS" && result.data) {
        // 토큰들을 LocalStorage에 저장
        localStorage.setItem('accessToken', result.data.accessToken);
        localStorage.setItem('refreshToken', result.data.refreshToken);
        
        // 백엔드 응답 구조에 맞게 사용자 정보 생성
        const userInfo = {
          userId: result.data.userId,
          username: result.data.username,
          name: result.data.username, // 백엔드에서 name 필드를 제공하지 않으므로 username 사용
          role: result.data.role,
          lastLoginAt: new Date().toISOString(), // 현재 로그인 시간으로 설정
          createdAt: null, // 로그인 응답에는 없음
          updatedAt: null  // 로그인 응답에는 없음
        };
        localStorage.setItem('user', JSON.stringify(userInfo));

        // 역할별 기본 페이지로 이동
        const defaultPage = authService.getDefaultPageByRole(result.data.role);
        navigate(defaultPage);
      } else {
        // API 명세서에 따른 에러 처리
        if (result.message?.includes('INVALID_CREDENTIALS')) {
          setError("잘못된 사용자명 또는 비밀번호입니다.");
        } else if (result.message?.includes('ACCOUNT_DISABLED')) {
          setError("비활성화된 계정입니다.");
        } else {
          setError(result.message || "로그인에 실패했습니다");
        }
      }
    } catch (err) {
      setError("네트워크 오류가 발생했습니다.");
      console.error('로그인 실패:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form className="login-form" onSubmit={handleSubmit} noValidate>
      <BrandLogo />
      
      {error && (
        <div className="error-message" style={{ color: 'red', marginBottom: '1rem' }}>
          {error}
        </div>
      )}
      
      <div className="field">
        <label htmlFor="username">사용자명</label>
        <input
          id="username"
          name="username"
          type="text"
          placeholder="사용자명을 입력하세요"
          value={formData.username}
          onChange={handleInputChange}
          required
          autoComplete="username"
        />
      </div>

      <div className="field">
        <label htmlFor="password">비밀번호</label>
        <input
          id="password"
          name="password"
          type="password"
          placeholder="비밀번호를 입력하세요"
          value={formData.password}
          onChange={handleInputChange}
          required
          autoComplete="current-password"
        />
      </div>

      <button type="submit" className="submit-btn" disabled={isLoading}>
        {isLoading ? '로그인 중...' : '로그인'}
      </button>
    </form>
  );
}
