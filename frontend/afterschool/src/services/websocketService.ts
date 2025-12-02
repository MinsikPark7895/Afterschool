import SockJS from 'sockjs-client';
import { Client, IMessage } from '@stomp/stompjs';

export interface RobotPositionUpdate {
  robotId: string;
  position: {
    x: number;
    y: number;
    z: number;
    pixel_x?: number;  // 백엔드에서 변환된 픽셀 좌표
    pixel_y?: number;  // 백엔드에서 변환된 픽셀 좌표
  };
  timestamp: string;
  batteryLevel?: number;
  state?: string;
  sensors?: {
    temperature?: number;
    humidity?: number;
    [key: string]: any;
  };
}

export interface WebSocketCallbacks {
  onRobotPositionUpdate?: (data: RobotPositionUpdate) => void;
  onEventAlert?: (data: any) => void;
  onCommandResult?: (data: any) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onError?: (error: any) => void;
}

class WebSocketService {
  private stompClient: Client | null = null;
  private isConnected: boolean = false;
  private callbacks: WebSocketCallbacks = {};
  private reconnectAttempts: number = 0;
  private maxReconnectAttempts: number = 5;
  private reconnectInterval: number = 3000; // 3초
  private baseUrl: string;

  constructor() {
    this.baseUrl = process.env.REACT_APP_API_URL || 'http://localhost:8080';
    // /api 제거하여 WebSocket 엔드포인트에 맞춤
    if (this.baseUrl.endsWith('/api')) {
      this.baseUrl = this.baseUrl.slice(0, -4);
    }
  }

  /**
   * WebSocket 연결 설정
   */
  connect(callbacks: WebSocketCallbacks): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.callbacks = callbacks;

        // SockJS 소켓 생성
        const socket = new SockJS(`${this.baseUrl}/api/ws`);
        
        // STOMP 클라이언트 생성
        this.stompClient = new Client({
          webSocketFactory: () => socket,
          debug: (str) => {
            console.log('STOMP Debug:', str);
          },
          reconnectDelay: this.reconnectInterval,
          heartbeatIncoming: 4000,
          heartbeatOutgoing: 4000,
        });

        // 연결 성공 핸들러
        this.stompClient.onConnect = (frame) => {
          console.log('WebSocket Connected:', frame);
          this.isConnected = true;
          this.reconnectAttempts = 0;

          // 로봇 위치 구독 (tb1, tb2)
          this.subscribeToRobotPositions();

          // 이벤트 알림 구독
          this.subscribeToEventAlerts();

          // 명령 결과 구독
          this.subscribeToCommandResults();

          if (this.callbacks.onConnect) {
            this.callbacks.onConnect();
          }

          resolve();
        };

        // 연결 오류 핸들러
        this.stompClient.onStompError = (frame) => {
          console.error('STOMP Error:', frame);
          this.isConnected = false;

          if (this.callbacks.onError) {
            this.callbacks.onError(frame);
          }

          // 재연결 시도
          this.handleReconnect();
          reject(new Error(`STOMP Error: ${frame.headers['message']}`));
        };

        // 연결 끊김 핸들러
        this.stompClient.onDisconnect = () => {
          console.log('WebSocket Disconnected');
          this.isConnected = false;

          if (this.callbacks.onDisconnect) {
            this.callbacks.onDisconnect();
          }

          // 재연결 시도
          this.handleReconnect();
        };

        // WebSocket 오류 핸들러
        this.stompClient.onWebSocketError = (error) => {
          console.error('WebSocket Error:', error);
          if (this.callbacks.onError) {
            this.callbacks.onError(error);
          }
          reject(error);
        };

        // 연결 활성화
        this.stompClient.activate();

      } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        reject(error);
      }
    });
  }

  /**
   * ROS 좌표를 픽셀 좌표로 변환하는 함수
   * 백엔드에서 변환하지 않은 경우 프론트엔드에서 변환
   */
  private rosToPixel(rosX: number, rosY: number): { pixel_x: number; pixel_y: number } {
    // 맵 설정 (백엔드 CoordinateTransformService와 동일한 로직 - application.yml과 동기화)
    const MAP_CONFIG = {
      resolution: 0.05,  // 미터/픽셀
      originX: -10.0,    // 맵 원점 X (미터)
      originY: -10.0,    // 맵 원점 Y (미터)
      width: 2062,       // 맵 너비 (픽셀) - 백엔드 application.yml과 동기화
      height: 893        // 맵 높이 (픽셀) - 백엔드 application.yml과 동기화
    };

    // ROS 좌표 → 픽셀 좌표 변환
    const pixel_x = (rosX - MAP_CONFIG.originX) / MAP_CONFIG.resolution;
    const pixel_y = MAP_CONFIG.height - ((rosY - MAP_CONFIG.originY) / MAP_CONFIG.resolution);

    console.log(`🔄 좌표 변환: ROS(${rosX}, ${rosY}) → 픽셀(${pixel_x.toFixed(1)}, ${pixel_y.toFixed(1)})`);

    return { pixel_x, pixel_y };
  }

  /**
   * 로봇 위치 구독
   */
  private subscribeToRobotPositions(): void {
    if (!this.stompClient || !this.isConnected) return;

    // tb1 로봇 위치 구독
    this.stompClient.subscribe('/topic/robot/basic/tb1', (message: IMessage) => {
      try {
        const data = JSON.parse(message.body);
        console.log('TB1 Position Update:', data);
        
        if (this.callbacks.onRobotPositionUpdate) {
          // 백엔드에서 오는 메시지 구조에 맞춰 파싱
          const payload = data.payload || data;
          
          // ROS 좌표 추출
          const rosX = payload.position?.x || data.x || 0;
          const rosY = payload.position?.y || data.y || 0;
          const rosZ = payload.position?.z || data.z || 0;

          // 픽셀 좌표가 없으면 ROS 좌표를 변환
          let pixelX = payload.position?.pixel_x;
          let pixelY = payload.position?.pixel_y;
          
          if (pixelX === undefined || pixelY === undefined) {
            const converted = this.rosToPixel(rosX, rosY);
            pixelX = converted.pixel_x;
            pixelY = converted.pixel_y;
            console.log(`📍 TB1 좌표 변환 완료: ROS(${rosX}, ${rosY}) → 픽셀(${pixelX.toFixed(1)}, ${pixelY.toFixed(1)})`);
          }
          
          this.callbacks.onRobotPositionUpdate({
            robotId: 'tb1',
            position: {
              x: rosX,           // 실제 ROS 좌표 보존
              y: rosY,
              z: rosZ,
              pixel_x: pixelX,   // 변환된 픽셀 좌표 추가
              pixel_y: pixelY
            },
            timestamp: data.header?.timestamp || data.timestamp || new Date().toISOString(),
            batteryLevel: payload.battery_level || payload.batteryLevel || data.batteryLevel,
            state: payload.state || data.state,
            sensors: payload.sensors
          });
        }
      } catch (error) {
        console.error('Error parsing TB1 position data:', error);
      }
    });

    // tb2 로봇 위치 구독
    this.stompClient.subscribe('/topic/robot/basic/tb2', (message: IMessage) => {
      try {
        const data = JSON.parse(message.body);
        console.log('TB2 Position Update:', data);
        
        if (this.callbacks.onRobotPositionUpdate) {
          // 백엔드에서 오는 메시지 구조에 맞춰 파싱
          const payload = data.payload || data;
          
          // ROS 좌표 추출
          const rosX = payload.position?.x || data.x || 0;
          const rosY = payload.position?.y || data.y || 0;
          const rosZ = payload.position?.z || data.z || 0;

          // 픽셀 좌표가 없으면 ROS 좌표를 변환
          let pixelX = payload.position?.pixel_x;
          let pixelY = payload.position?.pixel_y;
          
          if (pixelX === undefined || pixelY === undefined) {
            const converted = this.rosToPixel(rosX, rosY);
            pixelX = converted.pixel_x;
            pixelY = converted.pixel_y;
            console.log(`📍 TB2 좌표 변환 완료: ROS(${rosX}, ${rosY}) → 픽셀(${pixelX.toFixed(1)}, ${pixelY.toFixed(1)})`);
          }
          
          this.callbacks.onRobotPositionUpdate({
            robotId: 'tb2',
            position: {
              x: rosX,           // 실제 ROS 좌표 보존
              y: rosY,
              z: rosZ,
              pixel_x: pixelX,   // 변환된 픽셀 좌표 추가
              pixel_y: pixelY
            },
            timestamp: data.header?.timestamp || data.timestamp || new Date().toISOString(),
            batteryLevel: payload.battery_level || payload.batteryLevel || data.batteryLevel,
            state: payload.state || data.state,
            sensors: payload.sensors
          });
        }
      } catch (error) {
        console.error('Error parsing TB2 position data:', error);
      }
    });

    console.log('Subscribed to robot positions for tb1 and tb2');
  }

  /**
   * 이벤트 알림 구독
   */
  private subscribeToEventAlerts(): void {
    if (!this.stompClient || !this.isConnected) return;

    this.stompClient.subscribe('/topic/event/alert', (message: IMessage) => {
      try {
        const data = JSON.parse(message.body);
        console.log('Event Alert:', data);
        
        if (this.callbacks.onEventAlert) {
          this.callbacks.onEventAlert(data);
        }
      } catch (error) {
        console.error('Error parsing event alert data:', error);
      }
    });

    console.log('Subscribed to event alerts');
  }

  /**
   * 명령 결과 구독
   */
  private subscribeToCommandResults(): void {
    if (!this.stompClient || !this.isConnected) return;

    this.stompClient.subscribe('/topic/command/result', (message: IMessage) => {
      try {
        const data = JSON.parse(message.body);
        console.log('Command Result:', data);
        
        if (this.callbacks.onCommandResult) {
          this.callbacks.onCommandResult(data);
        }
      } catch (error) {
        console.error('Error parsing command result data:', error);
      }
    });

    console.log('Subscribed to command results');
  }

  /**
   * 재연결 처리
   */
  private handleReconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('Max reconnection attempts reached');
      return;
    }

    this.reconnectAttempts++;
    console.log(`Attempting to reconnect... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);

    setTimeout(() => {
      if (!this.isConnected && this.stompClient) {
        this.stompClient.activate();
      }
    }, this.reconnectInterval);
  }

  /**
   * 연결 끊기
   */
  disconnect(): void {
    if (this.stompClient) {
      this.stompClient.deactivate();
      this.stompClient = null;
    }
    this.isConnected = false;
    this.reconnectAttempts = 0;
    console.log('WebSocket disconnected manually');
  }

  /**
   * 연결 상태 확인
   */
  isConnectedToServer(): boolean {
    return this.isConnected;
  }

  /**
   * 메시지 전송 (필요시 사용)
   */
  sendMessage(destination: string, body: any): void {
    if (this.stompClient && this.isConnected) {
      this.stompClient.publish({
        destination,
        body: JSON.stringify(body)
      });
    } else {
      console.warn('WebSocket is not connected. Cannot send message.');
    }
  }
}

// 싱글톤 인스턴스 생성
export const websocketService = new WebSocketService();
