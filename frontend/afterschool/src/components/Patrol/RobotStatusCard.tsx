import React from "react";
import "./RobotStatusCard.css";

interface RobotStatusCardProps {
    robotId: string;
    robotName: string;
    battery: number;
    state: string;        // mission 대신 state 사용 (MQTT 데이터 구조에 맞춤)
    isOnline: boolean;
    isActive: boolean;    // 로봇 활성화 상태 (DB에서)
    lastUpdate: Date;
}

const RobotStatusCard: React.FC<RobotStatusCardProps> = ({
    robotId,
    robotName,
    battery,
    state,
    isOnline,
    isActive,
    lastUpdate
}) => {

    const formatLastUpdate = (date: Date) => {
        const now = new Date();
        const diffMs = now.getTime() - date.getTime();
        const diffMins = Math.floor(diffMs / 60000);
        
        if (diffMins < 1) return '방금 전';
        if (diffMins < 60) return `${diffMins}분 전`;
        const diffHours = Math.floor(diffMins / 60);
        if (diffHours < 24) return `${diffHours}시간 전`;
        return date.toLocaleDateString();
    };
    return (
        <div className={`robot-status-card ${isOnline ? 'online' : 'offline'}`}>
            <div className='card-header'>
                <h3 className='robot-name'>{robotName}</h3>
                <div className='status-indicators'>
                    <span className={`zone-badge ${isOnline ? 'online' : 'offline'}`}>
                        Zone1
                    </span>
                    <span className={`connection-status ${isOnline ? 'online' : 'offline'}`}>
                        {isOnline ? '온라인' : '오프라인'}
                    </span>
                </div>
            </div>
            <div className='card-content'>
                <div className='status-item'>
                    <span className='status-label'>배터리</span>
                    <span className='status-value battery'>{battery > 0 ? `${battery}%` : '알 수 없음'}</span>
                </div>
                <div className='status-item'>
                    <span className='status-label'>상태</span>
                    <span className='status-value state'>{state || '알 수 없음'}</span>
                </div>                
                {lastUpdate && (
                    <div className='status-item'>
                        <span className='status-label'>마지막 업데이트</span>
                        <span className='status-value'>{formatLastUpdate(lastUpdate)}</span>
                    </div>
                )}
            </div>
        </div>
    );
};


export default RobotStatusCard;