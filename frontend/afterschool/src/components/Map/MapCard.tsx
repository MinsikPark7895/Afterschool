interface MapCardProps {
  id: number;
  mapId: string;
  mapName: string;
  coveragePercentage: number;
  isActive: boolean;
  onClick?: () => void;
}

export default function MapCard({
  id,
  mapId,
  mapName,
  coveragePercentage,
  isActive,
  onClick,
}: MapCardProps) {
  return (
    <div className="map-item" onClick={onClick}>
      <div className="map-image">
        <div className="placeholder" />
      </div>
      <div className="map-label">
        {id}. {coveragePercentage}% {mapName}
      </div>
    </div>
  );
}
