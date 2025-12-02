import React from 'react';

interface ProfileButtonProps {
  onClick: () => void;
}

const ProfileButton: React.FC<ProfileButtonProps> = ({ onClick }) => {
  return (
    <button 
      className="nav-link"
      onClick={onClick}
      style={{ background: 'none', border: 'none', cursor: 'pointer' }}
    >
      프로필
    </button>
  );
};

export default ProfileButton;
