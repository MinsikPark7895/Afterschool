import React, { useState, useEffect } from 'react';
import EventList from '../components/Event/EventList';
import { EventListRequest } from '../types/event';
import './EventPage.css';
import Header from '../components/Header/Header';

const EventPage: React.FC = () => {
  const [selectedFilters, setSelectedFilters] = useState<Partial<EventListRequest>>({});

  // URL 파라미터에서 초기 필터 설정 (선택사항)
  useEffect(() => {
    const urlParams = new URLSearchParams(window.location.search);
    const initialFilters: Partial<EventListRequest> = {};
    
    if (urlParams.get('robotId')) {
      initialFilters.robotId = urlParams.get('robotId') || undefined;
    }
    if (urlParams.get('severity')) {
      initialFilters.severity = urlParams.get('severity') as any;
    }
    if (urlParams.get('startDate')) {
      initialFilters.startDate = urlParams.get('startDate') || undefined;
    }
    if (urlParams.get('endDate')) {
      initialFilters.endDate = urlParams.get('endDate') || undefined;
    }
    
    if (Object.keys(initialFilters).length > 0) {
      setSelectedFilters(initialFilters);
    }
  }, []);

  return (
    <div className="event-page">
      <Header />

      <div className="event-page-content">
        <EventList 
          className="main-event-list"
          initialFilters={selectedFilters}
        />
      </div>
    </div>
  );
};

export default EventPage;
