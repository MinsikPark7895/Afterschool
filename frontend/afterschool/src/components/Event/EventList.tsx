import React, { useState, useEffect } from 'react';
import { Event, EventListRequest } from '../../types/event';
import { eventService } from '../../services/eventService';
import EventCard from './EventCard';
import EventFilter from './EventFilter';
import './EventList.css';

interface EventListProps {
  className?: string;
  initialFilters?: Partial<EventListRequest>;
}

const EventList: React.FC<EventListProps> = ({ className, initialFilters = {} }) => {
  const [events, setEvents] = useState<Event[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [filters, setFilters] = useState<EventListRequest>({
    page: 0,
    size: 20,
    ...initialFilters
  });
  const [pagination, setPagination] = useState({
    totalElements: 0,
    totalPages: 0,
    first: true,
    last: true
  });

  // 이벤트 목록 로드
  const loadEvents = async (newFilters?: EventListRequest) => {
    setLoading(true);
    setError(null);
    
    try {
      const currentFilters = newFilters || filters;
      const response = await eventService.getEvents(currentFilters);
      
      if (response.status === 'SUCCESS') {
        setEvents(response.data.content);
        setPagination({
          totalElements: response.data.totalElements,
          totalPages: response.data.totalPages,
          first: response.data.first,
          last: response.data.last
        });
      } else {
        setError(response.message || '이벤트 목록을 불러오는데 실패했습니다.');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : '이벤트 목록을 불러오는데 실패했습니다.');
    } finally {
      setLoading(false);
    }
  };

  // 컴포넌트 마운트 시 이벤트 로드
  useEffect(() => {
    loadEvents();
  }, []);

  // 필터 변경 핸들러
  const handleFilterChange = (newFilters: Partial<EventListRequest>) => {
    const updatedFilters = { ...filters, ...newFilters, page: 0 };
    setFilters(updatedFilters);
    loadEvents(updatedFilters);
  };

  // 페이지 변경 핸들러
  const handlePageChange = (page: number) => {
    const updatedFilters = { ...filters, page };
    setFilters(updatedFilters);
    loadEvents(updatedFilters);
  };

  // 새로고침 핸들러
  const handleRefresh = () => {
    loadEvents();
  };


  return (
    <div className={`event-list ${className || ''}`}>
      <div className="event-list-header">
        <h2>이벤트 목록</h2>
        <button 
          className="refresh-button" 
          onClick={handleRefresh}
          disabled={loading}
        >
          {loading ? '로딩중...' : '새로고침'}
        </button>
      </div>

      {/* 필터 컴포넌트 */}
      <EventFilter 
        filters={filters}
        onFilterChange={handleFilterChange}
      />

      {/* 에러 메시지 */}
      {error && (
        <div className="error-message">
          {error}
        </div>
      )}

      {/* 로딩 상태 */}
      {loading && (
        <div className="loading-message">
          이벤트를 불러오는 중...
        </div>
      )}

      {/* 이벤트 목록 */}
      {!loading && events.length === 0 ? (
        <div className="no-events-message">
          조건에 맞는 이벤트가 없습니다.
        </div>
      ) : (
        <>
          <div className="event-cards">
            {events.map((event) => (
              <EventCard 
                key={event.id} 
                event={event}
                onRefresh={handleRefresh}
              />
            ))}
          </div>

          {/* 페이지네이션 */}
          {pagination.totalPages > 1 && (
            <div className="pagination">
              <button 
                onClick={() => handlePageChange(filters.page! - 1)}
                disabled={pagination.first || loading}
              >
                이전
              </button>
              
              <span className="page-info">
                {filters.page! + 1} / {pagination.totalPages} 페이지
                (총 {pagination.totalElements}개)
              </span>
              
              <button 
                onClick={() => handlePageChange(filters.page! + 1)}
                disabled={pagination.last || loading}
              >
                다음
              </button>
            </div>
          )}
        </>
      )}
    </div>
  );
};

export default EventList;
