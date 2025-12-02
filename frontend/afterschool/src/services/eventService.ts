import { 
  EventListRequest, 
  EventListResponse, 
  EventDetailResponse, 
  EvidenceFileListResponse,
  FileDownloadResponse 
} from '../types/event';

const BASE_URL = process.env.REACT_APP_API_URL || '/api';

export const eventService = {
  /**
   * 침입자 탐지 이벤트 목록 조회
   * @param params 조회 조건 (심각도, 로봇ID, 날짜 범위, 페이지 정보)
   * @returns 이벤트 목록 응답
   */
  async getEvents(params: EventListRequest = {}): Promise<EventListResponse> {
    const queryParams = new URLSearchParams();
    
    // 쿼리 파라미터 구성
    if (params.severity) queryParams.append('severity', params.severity);
    if (params.robotId) queryParams.append('robotId', params.robotId);
    if (params.startDate) queryParams.append('startDate', params.startDate);
    if (params.endDate) queryParams.append('endDate', params.endDate);
    if (params.page !== undefined) queryParams.append('page', params.page.toString());
    if (params.size !== undefined) queryParams.append('size', params.size.toString());

    const response = await fetch(`${BASE_URL}/events?${queryParams}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  /**
   * 특정 이벤트의 상세 정보 조회
   * @param eventId 이벤트 ID
   * @returns 이벤트 상세 정보
   */
  async getEventDetail(eventId: number): Promise<EventDetailResponse> {
    const response = await fetch(`${BASE_URL}/events/${eventId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error('EVENT_NOT_FOUND');
      }
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  /**
   * 특정 이벤트의 증거 파일 목록 조회
   * @param eventId 이벤트 ID
   * @returns 증거 파일 목록
   */
  async getEvidenceFiles(eventId: number): Promise<EvidenceFileListResponse> {
    const response = await fetch(`${BASE_URL}/events/evidence/${eventId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error('EVENT_NOT_FOUND');
      }
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  /**
   * S3에 저장된 증거 파일 다운로드
   * @param fileId 증거 파일 ID
   * @returns 파일 다운로드 응답 (Blob)
   */
  async downloadEvidenceFile(fileId: number): Promise<FileDownloadResponse> {
    const response = await fetch(`${BASE_URL}/events/evidence/download/${fileId}`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error('FILE_NOT_FOUND');
      }
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    // Content-Disposition 헤더에서 파일명 추출
    const contentDisposition = response.headers.get('Content-Disposition');
    let filename = `evidence_file_${fileId}`;
    
    if (contentDisposition) {
      const filenameMatch = contentDisposition.match(/filename="?([^"]+)"?/);
      if (filenameMatch) {
        filename = filenameMatch[1];
      }
    }

    const blob = await response.blob();
    
    return {
      blob,
      filename
    };
  },

  /**
   * 파일 다운로드 헬퍼 함수
   * 브라우저에서 파일을 자동으로 다운로드합니다.
   * @param fileId 증거 파일 ID
   */
  async downloadAndSaveFile(fileId: number): Promise<void> {
    try {
      const { blob, filename } = await this.downloadEvidenceFile(fileId);
      
      // Blob URL 생성
      const url = window.URL.createObjectURL(blob);
      
      // 임시 링크 생성 및 클릭
      const link = document.createElement('a');
      link.href = url;
      link.download = filename;
      document.body.appendChild(link);
      link.click();
      
      // 정리
      document.body.removeChild(link);
      window.URL.revokeObjectURL(url);
    } catch (error) {
      console.error('파일 다운로드 실패:', error);
      throw error;
    }
  },

  /**
   * 날짜 범위로 이벤트 조회 (편의 메서드)
   * @param startDate 시작 날짜 (YYYY-MM-DD)
   * @param endDate 종료 날짜 (YYYY-MM-DD)
   * @param severity 심각도 필터 (선택사항)
   * @param robotId 로봇 ID 필터 (선택사항)
   * @returns 이벤트 목록
   */
  async getEventsByDateRange(
    startDate: string, 
    endDate: string, 
    severity?: string, 
    robotId?: string
  ): Promise<EventListResponse> {
    return this.getEvents({
      startDate,
      endDate,
      severity: severity as any,
      robotId,
      page: 0,
      size: 20
    });
  },

  /**
   * 특정 로봇의 이벤트 조회 (편의 메서드)
   * @param robotId 로봇 ID
   * @param page 페이지 번호 (기본값: 0)
   * @param size 페이지 크기 (기본값: 20)
   * @returns 이벤트 목록
   */
  async getEventsByRobot(
    robotId: string, 
    page: number = 0, 
    size: number = 20
  ): Promise<EventListResponse> {
    return this.getEvents({
      robotId,
      page,
      size
    });
  },

  /**
   * 심각도별 이벤트 조회 (편의 메서드)
   * @param severity 심각도
   * @param page 페이지 번호 (기본값: 0)
   * @param size 페이지 크기 (기본값: 20)
   * @returns 이벤트 목록
   */
  async getEventsBySeverity(
    severity: string, 
    page: number = 0, 
    size: number = 20
  ): Promise<EventListResponse> {
    return this.getEvents({
      severity: severity as any,
      page,
      size
    });
  }
};
