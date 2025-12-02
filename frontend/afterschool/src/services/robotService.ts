import { RobotListResponse, RobotDetailResponse, PatrolStartRequest, CommandResponse, PatrolStopRequest, MoveToRequest, EmergencyStopRequest } from '../types/robot';

const BASE_URL = process.env.REACT_APP_API_URL || '/api';

export const robotService = {
  // 로봇 목록 조회 (새로운 API 스펙에 맞춤)
  async getRobots(): Promise<RobotListResponse> {
    const response = await fetch(`${BASE_URL}/robots`, {
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

  // 특정 로봇 조회
  async getRobotDetail(robotId: string): Promise<RobotDetailResponse> {
    const response = await fetch(`${BASE_URL}/robots/${robotId}`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error('ROBOT_NOT_FOUND');
      }
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 순찰 시작
  async startPatrol(robotId: string, patrolData: PatrolStartRequest): Promise<CommandResponse> {
    const response = await fetch(`${BASE_URL}/robots/${robotId}/start-patrol`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      },
      body: JSON.stringify(patrolData)
    });

    if (!response.ok) {
      if (response.status === 409) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'ROBOT_CONFLICT');  
      }
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 순찰 중지
  async stopPatrol(robotId: string, stopData: PatrolStopRequest): Promise<CommandResponse> {
    const response = await fetch(`${BASE_URL}/robots/${robotId}/stop-patrol`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      },
      body: JSON.stringify(stopData)
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 특정 위치로 이동
  async moveTo(robotId: string, moveData: MoveToRequest): Promise<CommandResponse> {
    const response = await fetch(`${BASE_URL}/robots/${robotId}/move-to`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      },
      body: JSON.stringify(moveData)
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 전체 로봇 순찰 시작
  async startAllPatrol(): Promise<CommandResponse> {
    const response = await fetch(`${BASE_URL}/robots/start-patrol`, {
      method: 'POST',
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

  // 전체 로봇 순찰 중지
  async stopAllPatrol(): Promise<CommandResponse> {
    const response = await fetch(`${BASE_URL}/robots/stop-patrol`, {
      method: 'POST',
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

  // 특정 로봇 이동 명령
  async moveRobotTo(robotId: string, targetPosition: { x: number; y: number; z?: number }): Promise<CommandResponse> {
    const response = await fetch(`${BASE_URL}/robots/move-to/${robotId}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      },
      body: JSON.stringify({
        targetPosition: {
          x: targetPosition.x,
          y: targetPosition.y,
          z: targetPosition.z || 0
        }
      })
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 전체 명령 로그 조회
  async getCommandLogs(params: {
    page?: number;
    size?: number;
    robotId?: string;
    commandType?: string;
    status?: string;
  } = {}): Promise<any> {
    const queryParams = new URLSearchParams();
    
    if (params.page !== undefined) queryParams.append('page', params.page.toString());
    if (params.size !== undefined) queryParams.append('size', params.size.toString());
    if (params.robotId) queryParams.append('robotId', params.robotId);
    if (params.commandType) queryParams.append('commandType', params.commandType);
    if (params.status) queryParams.append('status', params.status);

    const response = await fetch(`${BASE_URL}/command-logs?${queryParams}`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 로봇별 명령 로그 조회
  async getCommandLogsByRobot(robotId: string, page: number = 0, size: number = 20): Promise<any> {
    const response = await fetch(`${BASE_URL}/command-logs/robot/${robotId}?page=${page}&size=${size}`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 명령 타입별 로그 조회
  async getCommandLogsByType(commandType: string, page: number = 0, size: number = 20): Promise<any> {
    const response = await fetch(`${BASE_URL}/command-logs/type/${commandType}?page=${page}&size=${size}`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
      }
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  },

  // 전체 로봇 긴급 정지
  async emergencyStop(stopData: EmergencyStopRequest): Promise<CommandResponse> {
    try {
      const response = await fetch(`${BASE_URL}/robots/emergency-stop`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
        },
        body: JSON.stringify(stopData)
      });

      if (!response.ok) {
        if (response.status === 405) {
          throw new Error('긴급정지 기능이 현재 지원되지 않습니다.');
        } else if (response.status === 401) {
          throw new Error('인증이 필요합니다. 다시 로그인해주세요.');
        } else if (response.status === 403) {
          throw new Error('긴급정지 권한이 없습니다.');
        }
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      if (error instanceof Error) {
        throw error;
      }
      throw new Error('긴급정지 요청 중 오류가 발생했습니다.');
    }
  }
};