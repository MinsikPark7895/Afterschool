import React from 'react';
import { Link } from 'react-router-dom';
import BrandLogo from "../../assets/icons/Logo/BrandLogo";

interface LogoSectionProps {
  to?: string;
}

const LogoSection: React.FC<LogoSectionProps> = ({ to = "/main" }) => {
  return (
    <Link to={to} className="logo-section">
      <BrandLogo />
    </Link>
  );
};

export default LogoSection;
