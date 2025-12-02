import React, { useState, useEffect } from 'react';
import { robotService } from '../../services/robotService';
import './CommandLogViewer.css';

interface CommandLog {
  id: number;
  targetRobotId: string;
  userId: number;
  commandType: string;
  commandPayload: string;
  commandStatus: string;
  responseData: string;
  sentAt: string;
  completedAt: string;
  executionTime: number;
}

interface CommandLogViewerProps {
  robotId?: string;
  commandType?: string;
  maxItems?: number;
}

const CommandLogViewer: React.FC<CommandLogViewerProps> = ({
  robotId,
  commandType,
  maxItems = 10
}) => {
  const [logs, setLogs] = useState<CommandLog[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [page, setPage] = useState(0);
  const [hasMore, setHasMore] = useState(true);

  const loadLogs = async (pageNum: number = 0, append: boolean = false) => {
    try {
      setLoading(true);
      setError(null);

      let response;
      
      if (robotId) {
        response = await robotService.getCommandLogsByRobot(robotId, pageNum, maxItems);
      } else if (commandType) {
        response = await robotService.getCommandLogsByType(commandType, pageNum, maxItems);
      } else {
        response = await robotService.getCommandLogs({
          page: pageNum,
          size: maxItems
        });
      }

      if (response.status === 'SUCCESS') {
        const newLogs = response.data.content;
        
        if (append) {
          setLogs(prev => [...prev, ...newLogs]);
        } else {
          setLogs(newLogs);
        }
        
        setHasMore(!response.data.last);
        setPage(pageNum);
      }
    } catch (err) {
      console.error('명령 로그 조회 실패:', err);
      setError('명령 로그를 불러오는데 실패했습니다.');
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    loadLogs(0, false);
  }, [robotId, commandType]);

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

  const getStatusBadge = (status: string) => {
    const statusConfig = {
      sent: { label: '전송됨', className: 'status-sent' },
      success: { label: '성공', className: 'status-success' },
      failed: { label: '실패', className: 'status-failed' }
    };

    const config = statusConfig[status.toLowerCase() as keyof typeof statusConfig] || 
                  { label: status, className: 'status-unknown' };

    return <span className={`status-badge ${config.className}`}>{config.label}</span>;
  };

  const getCommandTypeName = (type: string) => {
    const typeNames = {
      'start_patrol': '순찰 시작',
      'stop_patrol': '순찰 중지',
      'move_to': '이동 명령',
      'emergency_stop': '긴급 정지'
    };

    return typeNames[type as keyof typeof typeNames] || type;
  };

  const loadMoreLogs = () => {
    if (!loading && hasMore) {
      loadLogs(page + 1, true);
    }
  };

  if (error) {
    return (
      <div className="command-log-error">
        <p>{error}</p>
        <button onClick={() => loadLogs(0, false)}>다시 시도</button>
      </div>
    );
  }

  return (
    <div className="command-log-viewer">
      <div className="command-log-header">
        <h3>🔍 명령 실행 이력</h3>
        <button 
          className="refresh-btn"
          onClick={() => loadLogs(0, false)}
          disabled={loading}
        >
          🔄 새로고침
        </button>
      </div>

      {loading && page === 0 ? (
        <div className="command-log-loading">
          <div className="loading-spinner"></div>
          <p>명령 로그를 불러오는 중...</p>
        </div>
      ) : (
        <>
          <div className="command-log-list">
            {logs.length === 0 ? (
              <div className="no-logs">
                <p>명령 실행 이력이 없습니다.</p>
              </div>
            ) : (
              logs.map((log) => (
                <div key={log.id} className="command-log-item">
                  <div className="log-header">
                    <div className="log-info">
                      <span className="command-type">
                        {getCommandTypeName(log.commandType)}
                      </span>
                      {log.targetRobotId && (
                        <span className="robot-id">🤖 {log.targetRobotId}</span>
                      )}
                    </div>
                    <div className="log-status">
                      {getStatusBadge(log.commandStatus)}
                    </div>
                  </div>
                  
                  <div className="log-details">
                    <div className="log-time">
                      <span>📅 전송: {formatDate(log.sentAt)}</span>
                      {log.completedAt && (
                        <span>✅ 완료: {formatDate(log.completedAt)}</span>
                      )}
                    </div>
                    
                    {log.executionTime > 0 && (
                      <div className="execution-time">
                        ⏱️ 실행 시간: {log.executionTime}ms
                      </div>
                    )}
                    
                    {log.responseData && log.commandStatus === 'failed' && (
                      <div className="error-details">
                        <span className="error-label">❌ 오류:</span>
                        <span className="error-message">{log.responseData}</span>
                      </div>
                    )}
                  </div>
                </div>
              ))
            )}
          </div>

          {hasMore && (
            <div className="load-more-container">
              <button 
                className="load-more-btn"
                onClick={loadMoreLogs}
                disabled={loading}
              >
                {loading ? '로딩 중...' : '더 보기'}
              </button>
            </div>
          )}
        </>
      )}
    </div>
  );
};

export default CommandLogViewer;
