import React, { useState } from 'react';
import { Event, EventSeverity } from '../../types/event';
import { eventService } from '../../services/eventService';
import './EventCard.css';

interface EventCardProps {
  event: Event;
  onRefresh?: () => void;
}

const EventCard: React.FC<EventCardProps> = ({ event, onRefresh }) => {
  const [showDetailModal, setShowDetailModal] = useState(false);
  const [EventDetailModal, setEventDetailModal] = useState<React.ComponentType<any> | null>(null);

  // 동적 import로 EventDetailModal 로드
  const loadEventDetailModal = async () => {
    if (!EventDetailModal) {
      const module = await import('./EventDetailModal');
      setEventDetailModal(() => module.default);
    }
    setShowDetailModal(true);
  };

  // 심각도별 색상 및 라벨 (백엔드 Severity enum에 맞춤)
  const getSeverityInfo = (severity: EventSeverity) => {
    switch (severity) {
      case 'CRITICAL':
        return { color: '#ff4444', label: '심각', bgColor: 'rgba(255, 68, 68, 0.1)' };
      case 'WARNING':
        return { color: '#ff8800', label: '경고', bgColor: 'rgba(255, 136, 0, 0.1)' };
      case 'INFO':
        return { color: '#4A90E2', label: '정보', bgColor: 'rgba(74, 144, 226, 0.1)' };
      default:
        return { color: '#666666', label: '알 수 없음', bgColor: 'rgba(255, 255, 255, 0.05)' };
    }
  };

  // 이벤트 타입별 라벨 (백엔드 EventType enum에 맞춤)
  const getEventTypeLabel = (eventType: string) => {
    switch (eventType) {
      case 'DETECTION': return '침입자 탐지';
      case 'MISSION_DONE': return '임무 완료';
      default: return eventType;
    }
  };

  // 날짜 포맷팅
  const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    return date.toLocaleString('ko-KR', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
  };

  // 위치 데이터 파싱 (JSON 문자열인 경우)
  const parseLocationData = (locationData: string) => {
    try {
      const parsed = JSON.parse(locationData);
      if (parsed.zone) return parsed.zone;
      if (parsed.x !== undefined && parsed.y !== undefined) {
        return `X: ${parsed.x.toFixed(2)}, Y: ${parsed.y.toFixed(2)}`;
      }
      return locationData;
    } catch {
      return locationData;
    }
  };

  // 증거 파일 다운로드
  const handleDownloadFile = async (fileId: number) => {
    try {
      await eventService.downloadAndSaveFile(fileId);
    } catch (error) {
      console.error('파일 다운로드 실패:', error);
      alert('파일 다운로드에 실패했습니다.');
    }
  };

  const severityInfo = getSeverityInfo(event.severity);

  return (
    <>
      <div 
        className="event-card" 
        style={{ borderLeftColor: severityInfo.color, backgroundColor: severityInfo.bgColor }}
      >
        <div className="event-card-header">
          <div className="event-type-badge">
            {getEventTypeLabel(event.eventType)}
          </div>
          <div 
            className="severity-badge" 
            style={{ backgroundColor: severityInfo.color }}
          >
            {severityInfo.label}
          </div>
        </div>

        <div className="event-card-content">
          <div className="event-info-row">
            <strong>로봇 ID:</strong> {event.robotId}
          </div>
          
          <div className="event-info-row">
            <strong>위치:</strong> {parseLocationData(event.locationData)}
          </div>
          
          <div className="event-info-row">
            <strong>발생 시간:</strong> {formatDate(event.createdAt)}
          </div>

          {event.detectionData && (
            <div className="event-info-row">
              <strong>탐지 정보:</strong> 
              <span className="detection-data">{event.detectionData}</span>
            </div>
          )}

          {event.evidenceFiles && event.evidenceFiles.length > 0 && (
            <div className="evidence-files">
              <strong>증거 파일 ({event.evidenceFiles.length}개):</strong>
              <div className="file-list">
                {event.evidenceFiles.slice(0, 3).map((file) => (
                  <button
                    key={file.id}
                    className="file-download-btn"
                    onClick={() => handleDownloadFile(file.id)}
                    title={file.originalFilename}
                  >
                    📎 {file.originalFilename.length > 20 
                      ? `${file.originalFilename.substring(0, 20)}...` 
                      : file.originalFilename}
                  </button>
                ))}
                {event.evidenceFiles.length > 3 && (
                  <span className="more-files">
                    +{event.evidenceFiles.length - 3}개 더
                  </span>
                )}
              </div>
            </div>
          )}
        </div>

        <div className="event-card-actions">
          <button 
            className="detail-button"
            onClick={loadEventDetailModal}
          >
            상세 보기
          </button>
        </div>
      </div>

      {/* 상세 모달 */}
      {showDetailModal && EventDetailModal && (
        <EventDetailModal
          eventId={event.id}
          onClose={() => setShowDetailModal(false)}
          onRefresh={onRefresh}
        />
      )}
    </>
  );
};

export default EventCard;
