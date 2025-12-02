import React from 'react';
import { MapDetailResponse, Zone } from '../../types/map';


interface MapDetailModalProps {
    isOpen: boolean;
    onClose: () => void;
    mapDetail: MapDetailResponse['data'] | null;
    onActivate: (mapId: string) => void;
}

const MapDetailModal: React.FC<MapDetailModalProps> = ({
    isOpen,
    onClose,
    mapDetail,
    onActivate,
}) => {
    if (!isOpen || !mapDetail) return null;

    return (
        <div className="modal-overlay" onClick={onClose}>
            <div className="modal-content" onClick={(e) => e.stopPropagation()}>
                <div className="modal-header">
                    <h2>맵 상세 정보</h2>
                    <button onClick={onClose}>&times;</button>
                </div>
                <div className="modal-body">
                    {/* 맵 상세 정보 표시 */}
                    <div className="map-info">
                        <h3>{mapDetail.mapName}</h3>
                        <p>해상도: {mapDetail.resolution}</p>
                        <p>크기: {mapDetail.width} x {mapDetail.height}</p>
                        <p>커버리지: {mapDetail.coveragePercentage}%</p>
                        <p>상태: {mapDetail.isActive ? '활성' : '비활성'}</p>
                    </div>

                    {/* 파일 목록 */}
                    <div className="file-section">
                        <h4>파일 목록</h4>
                        {mapDetail.zones.map((zone: Zone) => (
                            <div key={zone.id} className="zone-item">
                                <span>{zone.zoneName}</span>
                                <span>로봇: {zone.assignedRobotId}</span>
                            </div>
                        ))}
                    </div>
                </div>
            </div>
        </div>
    )
}

export default MapDetailModal;