import React, { useState, useEffect } from 'react';
import { Event, EvidenceFile } from '../../types/event';
import { eventService } from '../../services/eventService';
import './EventDetailModal.css';

interface EventDetailModalProps {
  eventId: number;
  onClose: () => void;
  onRefresh?: () => void;
}

const EventDetailModal: React.FC<EventDetailModalProps> = ({ 
  eventId, 
  onClose, 
  onRefresh 
}) => {
  const [event, setEvent] = useState<Event | null>(null);
  const [evidenceFiles, setEvidenceFiles] = useState<EvidenceFile[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [activeTab, setActiveTab] = useState<'info' | 'files'>('info');

  // 이벤트 상세 정보 로드
  const loadEventDetail = async () => {
    setLoading(true);
    setError(null);
    
    try {
      const [eventResponse, filesResponse] = await Promise.all([
        eventService.getEventDetail(eventId),
        eventService.getEvidenceFiles(eventId)
      ]);

      if (eventResponse.status === 'SUCCESS') {
        setEvent(eventResponse.data);
      } else {
        setError(eventResponse.message || '이벤트 정보를 불러오는데 실패했습니다.');
      }

      if (filesResponse.status === 'SUCCESS') {
        setEvidenceFiles(filesResponse.data);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : '이벤트 정보를 불러오는데 실패했습니다.');
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    loadEventDetail();
  }, [eventId]);

  // 파일 다운로드
  const handleDownloadFile = async (fileId: number) => {
    try {
      await eventService.downloadAndSaveFile(fileId);
    } catch (error) {
      console.error('파일 다운로드 실패:', error);
      alert('파일 다운로드에 실패했습니다.');
    }
  };

  // 파일 크기 포맷팅
  const formatFileSize = (bytes: number): string => {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
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

  // 심각도별 색상 (백엔드 Severity enum에 맞춤)
  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'CRITICAL': return '#ff4444';
      case 'WARNING': return '#ff8800';
      case 'INFO': return '#4A90E2';
      default: return '#666666';
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

  // 위치 데이터 파싱
  const parseLocationData = (locationData: string) => {
    try {
      const parsed = JSON.parse(locationData);
      return (
        <div className="location-details">
          {parsed.zone && <div><strong>구역:</strong> {parsed.zone}</div>}
          {parsed.x !== undefined && <div><strong>X 좌표:</strong> {parsed.x}</div>}
          {parsed.y !== undefined && <div><strong>Y 좌표:</strong> {parsed.y}</div>}
          {parsed.z !== undefined && <div><strong>Z 좌표:</strong> {parsed.z}</div>}
        </div>
      );
    } catch {
      return <div>{locationData}</div>;
    }
  };

  // 탐지 데이터 파싱
  const parseDetectionData = (detectionData: string) => {
    try {
      const parsed = JSON.parse(detectionData);
      return (
        <div className="detection-details">
          {Object.entries(parsed).map(([key, value]) => (
            <div key={key}>
              <strong>{key}:</strong> {String(value)}
            </div>
          ))}
        </div>
      );
    } catch {
      return <div>{detectionData}</div>;
    }
  };

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="event-detail-modal" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h2>이벤트 상세 정보</h2>
          <button className="close-button" onClick={onClose}>×</button>
        </div>

        {loading && (
          <div className="modal-loading">
            이벤트 정보를 불러오는 중...
          </div>
        )}

        {error && (
          <div className="modal-error">
            {error}
          </div>
        )}

        {event && (
          <>
            <div className="modal-tabs">
              <button 
                className={`tab-button ${activeTab === 'info' ? 'active' : ''}`}
                onClick={() => setActiveTab('info')}
              >
                기본 정보
              </button>
              <button 
                className={`tab-button ${activeTab === 'files' ? 'active' : ''}`}
                onClick={() => setActiveTab('files')}
              >
                증거 파일 ({evidenceFiles.length})
              </button>
            </div>

            <div className="modal-content">
              {activeTab === 'info' && (
                <div className="event-info-tab">
                  <div className="info-section">
                    <h3>기본 정보</h3>
                    <div className="info-grid">
                      <div className="info-item">
                        <label>이벤트 ID</label>
                        <span>{event.id}</span>
                      </div>
                      <div className="info-item">
                        <label>이벤트 타입</label>
                        <span>{getEventTypeLabel(event.eventType)}</span>
                      </div>
                      <div className="info-item">
                        <label>심각도</label>
                        <span 
                          className="severity-badge"
                          style={{ backgroundColor: getSeverityColor(event.severity) }}
                        >
                          {event.severity}
                        </span>
                      </div>
                      <div className="info-item">
                        <label>로봇 ID</label>
                        <span>{event.robotId}</span>
                      </div>
                      <div className="info-item">
                        <label>발생 시간</label>
                        <span>{formatDate(event.createdAt)}</span>
                      </div>
                    </div>
                  </div>

                  <div className="info-section">
                    <h3>위치 정보</h3>
                    {parseLocationData(event.locationData)}
                  </div>

                  {event.detectionData && (
                    <div className="info-section">
                      <h3>탐지 정보</h3>
                      {parseDetectionData(event.detectionData)}
                    </div>
                  )}
                </div>
              )}

              {activeTab === 'files' && (
                <div className="evidence-files-tab">
                  {evidenceFiles.length === 0 ? (
                    <div className="no-files-message">
                      증거 파일이 없습니다.
                    </div>
                  ) : (
                    <div className="files-grid">
                      {evidenceFiles.map((file) => (
                        <div key={file.id} className="file-card">
                          <div className="file-info">
                            <div className="file-name" title={file.originalFilename}>
                              {file.originalFilename}
                            </div>
                            <div className="file-details">
                              <span>타입: {file.fileType}</span>
                              <span>크기: {formatFileSize(file.fileSize)}</span>
                              <span>업로드: {formatDate(file.createdAt)}</span>
                            </div>
                            {file.metadata && (
                              <div className="file-metadata">
                                메타데이터: {file.metadata}
                              </div>
                            )}
                          </div>
                          <button
                            className="download-button"
                            onClick={() => handleDownloadFile(file.id)}
                          >
                            다운로드
                          </button>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              )}
            </div>
          </>
        )}

        <div className="modal-actions">
          <button className="close-modal-button" onClick={onClose}>
            닫기
          </button>
        </div>
      </div>
    </div>
  );
};

export default EventDetailModal;
