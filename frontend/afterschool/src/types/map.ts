// 기존 타입들
export interface MapActivateResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: any;
}

export interface MapCreateRequest {
  mapName: string;
  description?: string;
}

export interface MapCreateResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: any;
}

export interface MapSaveRequest {
  mapId: string;
  mapData: any;
}

// Zone 인터페이스 추가
export interface Zone {
  id: string;
  zoneName: string;
  assignedRobotId: string;
}

// MapDetail 인터페이스 추가
export interface MapDetail {
  mapId: string;
  mapName: string;
  resolution: number;
  width: number;
  height: number;
  coveragePercentage: number;
  isActive: boolean;
  zones: Zone[];
}

export interface MapDetailResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: MapDetail;
}

export interface MapListResponse {
  status: "SUCCESS" | "ERROR";
  message: string;
  data?: any[];
}

// 새로운 타입 정의들
export interface MapConfig {
  resolution: number; // 미터당 픽셀 수
  origin: {
    x: number;
    y: number;
    yaw: number;
  };
  width: number; // 픽셀 단위
  height: number; // 픽셀 단위
  occupied_thresh?: number;
  free_thresh?: number;
  negate?: number;
}

export interface PGMFileInfo {
  fileName: string;
  url: string;
  configFile: string;
  blob?: Blob;
  isDirectUrl?: boolean; // S3 직접 링크인지 여부
}

// YAML 파싱을 위한 유틸리티 함수
export const parseYamlConfig = (yamlContent: string): MapConfig => {
  const lines = yamlContent.split('\n');
  const config: any = {};
  
  lines.forEach(line => {
    const trimmed = line.trim();
    if (trimmed && !trimmed.startsWith('#')) {
      const [key, value] = trimmed.split(':').map(s => s.trim());
      if (key && value) {
        if (key === 'origin') {
          // origin은 배열 형태로 파싱
          const originMatch = value.match(/\[(.*?)\]/);
          if (originMatch) {
            const [x, y, yaw] = originMatch[1].split(',').map(s => parseFloat(s.trim()));
            config.origin = { x, y, yaw };
          }
        } else if (['resolution', 'occupied_thresh', 'free_thresh', 'negate'].includes(key)) {
          config[key] = parseFloat(value);
        } else if (['width', 'height'].includes(key)) {
          config[key] = parseInt(value);
        } else {
          config[key] = value.replace(/['"]/g, '');
        }
      }
    }
  });
  
  return config as MapConfig;
};