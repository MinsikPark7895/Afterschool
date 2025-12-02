import LoginForm from "../components/Login/LoginForm";
import "./LoginPage.css";

export default function LoginPage() {
  return (
    <main role="main" className="login-page">
      <header className="login-header">
        <nav></nav>
      </header>
      <section className="login-card">
        <LoginForm />
      </section>
    </main>
  );
}
