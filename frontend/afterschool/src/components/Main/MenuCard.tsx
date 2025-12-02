import React from "react";
import { useNavigate } from "react-router-dom";
import "./MenuCard.css";

interface MenuCardProps {
  title: string;
  description: string;
  icon: string;
  route: string;
  color: string;
}

const MenuCard: React.FC<MenuCardProps> = ({
  title,
  description,
  icon,
  route,
  color,
}) => {
  const navigate = useNavigate();

  const handleClick = () => {
    navigate(route);
  };

  return (
    <div
      className="menu-card"
      onClick={handleClick}
      style={{ backgroundColor: color }}
    >
      <div className="menu-card-icon">{icon}</div>
      <div className="menu-card-content">
        <h3 className="menu-card-title">{title}</h3>
        <p className="menu-card-description">{description}</p>
      </div>
    </div>
  );
};

export default MenuCard;
