import React, { useState } from 'react';
import { EventListRequest, EventSeverity } from '../../types/event';
import './EventFilter.css';

interface EventFilterProps {
  filters: EventListRequest;
  onFilterChange: (filters: Partial<EventListRequest>) => void;
}

const EventFilter: React.FC<EventFilterProps> = ({ filters, onFilterChange }) => {
  const [isExpanded, setIsExpanded] = useState(false);

  // 오늘 날짜를 YYYY-MM-DD 형식으로 반환
  const getTodayString = () => {
    const today = new Date();
    return today.toISOString().split('T')[0];
  };

  // 일주일 전 날짜를 YYYY-MM-DD 형식으로 반환
  const getWeekAgoString = () => {
    const weekAgo = new Date();
    weekAgo.setDate(weekAgo.getDate() - 7);
    return weekAgo.toISOString().split('T')[0];
  };

  // 필터 초기화
  const handleReset = () => {
    onFilterChange({
      severity: undefined,
      robotId: undefined,
      startDate: undefined,
      endDate: undefined,
      page: 0
    });
  };

  // 빠른 필터 적용
  const applyQuickFilter = (type: string) => {
    const today = getTodayString();
    const weekAgo = getWeekAgoString();
    
    switch (type) {
      case 'today':
        onFilterChange({ startDate: today, endDate: today });
        break;
      case 'week':
        onFilterChange({ startDate: weekAgo, endDate: today });
        break;
      case 'critical':
        onFilterChange({ severity: 'CRITICAL' });
        break;
      case 'warning':
        onFilterChange({ severity: 'WARNING' });
        break;
    }
  };

  return (
    <div className="event-filter">
      <div className="filter-header">
        <button 
          className="filter-toggle"
          onClick={() => setIsExpanded(!isExpanded)}
        >
          필터 {isExpanded ? '▲' : '▼'}
        </button>
        
        <div className="quick-filters">
          <button 
            className="quick-filter-btn"
            onClick={() => applyQuickFilter('today')}
          >
            오늘
          </button>
          <button 
            className="quick-filter-btn"
            onClick={() => applyQuickFilter('week')}
          >
            최근 7일
          </button>
          <button 
            className="quick-filter-btn critical"
            onClick={() => applyQuickFilter('critical')}
          >
            심각
          </button>
          <button 
            className="quick-filter-btn warning"
            onClick={() => applyQuickFilter('warning')}
          >
            경고
          </button>
        </div>
      </div>

      {isExpanded && (
        <div className="filter-content">
          <div className="filter-row">
            <div className="filter-group">
              <label>심각도</label>
              <select 
                value={filters.severity || ''} 
                onChange={(e) => onFilterChange({ 
                  severity: e.target.value as EventSeverity || undefined 
                })}
              >
                <option value="">전체</option>
                <option value="CRITICAL">심각</option>
                <option value="WARNING">경고</option>
                <option value="INFO">정보</option>
              </select>
            </div>

            <div className="filter-group">
              <label>로봇 ID</label>
              <input
                type="text"
                value={filters.robotId || ''}
                onChange={(e) => onFilterChange({ robotId: e.target.value || undefined })}
                placeholder="로봇 ID 입력"
              />
            </div>

            <div className="filter-group">
              <label>시작 날짜</label>
              <input
                type="date"
                value={filters.startDate || ''}
                onChange={(e) => onFilterChange({ startDate: e.target.value || undefined })}
              />
            </div>

            <div className="filter-group">
              <label>종료 날짜</label>
              <input
                type="date"
                value={filters.endDate || ''}
                onChange={(e) => onFilterChange({ endDate: e.target.value || undefined })}
              />
            </div>

            <div className="filter-group">
              <label>페이지 크기</label>
              <select 
                value={filters.size || 20} 
                onChange={(e) => onFilterChange({ 
                  size: parseInt(e.target.value), 
                  page: 0 
                })}
              >
                <option value={10}>10개</option>
                <option value={20}>20개</option>
                <option value={50}>50개</option>
                <option value={100}>100개</option>
              </select>
            </div>
          </div>

          <div className="filter-actions">
            <button className="reset-button" onClick={handleReset}>
              초기화
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default EventFilter;
