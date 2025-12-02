import KaKaoIcon from "../../assets/icons/socialLogo/KakaoTalk_logo.svg";
import GoogleIcon from "../../assets/icons/socialLogo/google-logo.png";
import NaverIcon from "../../assets/icons/socialLogo/naver-logo.png";

export default function SocialLogo() {
  return (
    <section className="social-login">
      <ul className="social-list">
        <li>
          <button type="button" aria-label="카카오로 로그인">
            <img src={KaKaoIcon} alt="KaKao Logo" />
          </button>
        </li>
        <li>
          <button type="button" aria-label="구글로 로그인">
            <img src={GoogleIcon} alt="Google Logo" />
          </button>
        </li>
        <li>
          <button type="button" aria-label="네이버로 로그인">
            <img src={NaverIcon} alt="Naver Logo" />
          </button>
        </li>
      </ul>
    </section>
  );
}
