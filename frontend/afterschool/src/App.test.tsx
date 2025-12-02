import React from 'react';
import { render, screen } from '@testing-library/react';
import App from './App';

test('renders login page', () => {
  render(<App />);
  // 로그인 페이지의 요소들을 확인
  const usernameLabel = screen.getByText(/사용자명/i);
  const passwordLabel = screen.getByText(/비밀번호/i);
  const loginButton = screen.getByRole('button', { name: /로그인/i });
  
  expect(usernameLabel).toBeInTheDocument();
  expect(passwordLabel).toBeInTheDocument();
  expect(loginButton).toBeInTheDocument();
});
