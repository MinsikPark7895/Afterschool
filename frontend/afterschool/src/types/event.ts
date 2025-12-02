// 이벤트 심각도 타입 (백엔드 Severity enum과 일치)
export type EventSeverity = "CRITICAL" | "WARNING" | "INFO";

// 이벤트 타입 (백엔드 EventType enum과 일치)
export type EventType = "DETECTION" | "MISSION_DONE";

// 파일 타입
export interface EvidenceFile {
  id: number;
  eventId: number;
  fileType: string;
  s3Bucket: string;
  s3Key: string;
  originalFilename: string;
  fileSize: number;
  mimeType: string;
  metadata: string;
  createdAt: string;
}

// 이벤트 기본 정보
export interface Event {
  id: number;
  eventType: EventType;
  robotId: string;
  severity: EventSeverity;
  locationData: string;
  detectionData: string;
  createdAt: string;
  evidenceFiles: EvidenceFile[];
}

// 이벤트 목록 조회 요청 파라미터
export interface EventListRequest {
  severity?: EventSeverity;
  robotId?: string;
  startDate?: string; // YYYY-MM-DD 형식
  endDate?: string;   // YYYY-MM-DD 형식
  page?: number;
  size?: number;
}

// 페이지네이션 정보
export interface PageInfo {
  page: number;
  size: number;
  totalElements: number;
  totalPages: number;
  first: boolean;
  last: boolean;
}

// 이벤트 목록 응답 데이터
export interface EventListData {
  content: Event[];
  page: number;
  size: number;
  totalElements: number;
  totalPages: number;
  first: boolean;
  last: boolean;
}

// API 응답 기본 구조
export interface ApiResponse<T> {
  status: "SUCCESS" | "ERROR";
  message: string;
  data: T;
}

// 이벤트 목록 조회 응답
export interface EventListResponse extends ApiResponse<EventListData> {}

// 이벤트 상세 조회 응답
export interface EventDetailResponse extends ApiResponse<Event> {}

// 증거 파일 목록 조회 응답
export interface EvidenceFileListResponse extends ApiResponse<EvidenceFile[]> {}

// 파일 다운로드 응답 (blob 형태)
export interface FileDownloadResponse {
  blob: Blob;
  filename: string;
}
