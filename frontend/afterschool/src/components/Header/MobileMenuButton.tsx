import React from 'react';
import './MobileMenuButton.css';

interface MobileMenuButtonProps {
  isOpen: boolean;
  onClick: () => void;
}

const MobileMenuButton: React.FC<MobileMenuButtonProps> = ({ isOpen, onClick }) => {
  return (
    <button 
      className={`mobile-menu-button ${isOpen ? 'open' : ''}`}
      onClick={onClick}
      aria-label={isOpen ? '메뉴 닫기' : '메뉴 열기'}
    >
      <span className="hamburger-line"></span>
      <span className="hamburger-line"></span>
      <span className="hamburger-line"></span>
    </button>
  );
};

export default MobileMenuButton;
