import React, { useState, useEffect, useRef } from 'react';
import Header from '../components/Header/Header';
import PatrolMap from '../components/Patrol/PatrolMap';
import RobotStatusCard from '../components/Patrol/RobotStatusCard';
import ControlButtons from '../components/Patrol/ControlButtons';
import CommandLogViewer from '../components/CommandLog/CommandLogViewer';
import './PatrolModePage.css';
import { mapService } from '../services/mapService';
import { robotService } from '../services/robotService';
import { websocketService, RobotPositionUpdate, WebSocketCallbacks } from '../services/websocketService';
import { MapConfig, PGMFileInfo, parseYamlConfig } from '../types/map';
import { Robot as ApiRobot } from '../types/robot';

// 화면 표시용 로봇 인터페이스 (실제 백엔드/MQTT 데이터 구조 기반)
interface Robot {
  id: string;          // robotId (tb1, tb2)
  name: string;        // robotName
  realX: number;       // MQTT position.x (ROS 좌표)
  realY: number;       // MQTT position.y (ROS 좌표)
  screenX: number;     // MQTT position.pixel_x (픽셀 좌표)
  screenY: number;     // MQTT position.pixel_y (픽셀 좌표)
  battery: number;     // MQTT battery_level
  state: string;       // MQTT state (patrolling, idle, moving 등)
  isOnline: boolean;   // WebSocket 연결 상태
  isActive: boolean;   // 로봇 활성화 상태 (DB에서)
  lastUpdate: Date;    // 마지막 업데이트 시간
  isUpdating?: boolean; // 위치 업데이트 애니메이션용
  isIntruderDetected?: boolean; // 침입자 감지 상태
  intruderDetectedAt?: Date; // 침입자 감지 시간
}

const PatrolModePage: React.FC = () => {
  const [isPatrolActive, setIsPatrolActive] = useState(false);
  // 추가된 상태 변수
  const [mapConfig, setMapConfig] = useState<MapConfig | null>(null);
  const [pgmFileInfo, setPgmFileInfo] = useState<PGMFileInfo | null>(null);
  const [robots, setRobots] = useState<Robot[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [wsConnected, setWsConnected] = useState(false);
  const [wsError, setWsError] = useState<string | null>(null);
  
  // WebSocket 연결 상태 관리
  const wsInitialized = useRef(false);
  
  // 수동 조작 관련 상태
  const [isManualMode, setIsManualMode] = useState(false);
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null);
  const [targetPosition, setTargetPosition] = useState<{ x: number; y: number } | null>(null);
  
  // 샘플 mapId (현재 미사용)
  // const mapId = 'sample-map-id';

  // 좌표 변환 함수들 (ROS 실제 좌표 → 화면 픽셀 좌표) - 백엔드와 동일한 로직
  const realToScreen = (realX: number, realY: number, config: MapConfig) => {
    // 백엔드 CoordinateCalculator와 동일한 공식 사용
    const pixelX = (realX - config.origin.x) / config.resolution;
    const pixelY = config.height - ((realY - config.origin.y) / config.resolution);
    
    console.log(`🔄 ROS → 화면 좌표 변환: ros(${realX}, ${realY}) → pixel(${pixelX}, ${pixelY})`);
    
    return { x: pixelX, y: pixelY };
  };

  // 기본 로봇 데이터 (실제 백엔드 구조 기반)
  const defaultRobots: Robot[] = [
    {
      id: 'tb1',
      name: '순찰로봇 TB1',
      realX: 0, // MQTT에서 업데이트됨
      realY: 0,
      screenX: 0, // MQTT에서 업데이트됨
      screenY: 0,
      battery: 0, // MQTT에서 업데이트됨
      state: '연결 대기중', // MQTT에서 업데이트됨
      isOnline: false, // WebSocket 연결 시 true로 변경
      isActive: true,
      isUpdating: false,
      lastUpdate: new Date()
    },
    {
      id: 'tb2',
      name: '순찰로봇 TB2',
      realX: 0,
      realY: 0,
      screenX: 0,
      screenY: 0,
      battery: 0,
      state: '연결 대기중',
      isOnline: false,
      isActive: true,
      isUpdating: false,
      lastUpdate: new Date()
    }
  ];

  // 샘플 순찰 경로 데이터 (실제 좌표, 미터 단위)
  const samplePatrolRoute = [
    { realX: 2.5, realY: 7.0 },
    { realX: 5.0, realY: 7.0 },
    { realX: 6.0, realY: 5.0 },
    { realX: 6.0, realY: 4.5 },
    { realX: 4.0, realY: 3.0 },
    { realX: 2.0, realY: 3.0 },
    { realX: 2.0, realY: 5.0 },
    { realX: 2.5, realY: 7.0 }
  ];

  const handleStartPatrol = async () => {
    try {
      if (isPatrolActive) {
        // 순찰 중지
        console.log('순찰 중지 요청...');
        const response = await robotService.stopAllPatrol();
        if (response.status === 'SUCCESS') {
          setIsPatrolActive(false);
          console.log('✅ 순찰 중지 완료');
        }
      } else {
        // 순찰 시작
        console.log('순찰 시작 요청...');
        const response = await robotService.startAllPatrol();
        if (response.status === 'SUCCESS') {
          setIsPatrolActive(true);
          console.log('✅ 순찰 시작 완료');
        }
      }
    } catch (error) {
      console.error('순찰 제어 실패:', error);
      alert(`순찰 ${isPatrolActive ? '중지' : '시작'}에 실패했습니다.`);
    }
  };

  const handleManualControl = () => {
    if (isManualMode) {
      // 수동 조작 모드 해제
      setIsManualMode(false);
      setSelectedRobot(null);
      setTargetPosition(null);
      console.log('수동 조작 모드 해제');
    } else {
      // 수동 조작 모드 활성화
      setIsManualMode(true);
      // 기본적으로 첫 번째 온라인 로봇 선택
      const onlineRobot = robots.find(robot => robot.isOnline);
      if (onlineRobot) {
        setSelectedRobot(onlineRobot.id);
        console.log(`수동 조작 모드 활성화 - 선택된 로봇: ${onlineRobot.id}`);
      } else {
        console.warn('온라인 상태의 로봇이 없습니다.');
      }
    }
  };

  // 맵 클릭 핸들러 (수동 조작용)
  const handleMapClick = async (event: React.MouseEvent<HTMLDivElement>) => {
    if (!isManualMode || !selectedRobot || !mapConfig) return;

    const rect = event.currentTarget.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const clickY = event.clientY - rect.top;

    console.log(`🎯 맵 클릭 - 픽셀 좌표: (${clickX}, ${clickY})`);

    // 이전 목표 지점과의 거리 확인 (최소 30픽셀 이상 떨어져야 함)
    if (targetPosition) {
      const distance = Math.sqrt(
        Math.pow(clickX - targetPosition.x, 2) + Math.pow(clickY - targetPosition.y, 2)
      );
      
      if (distance < 30) {
        console.log('목표 지점 취소 - 너무 가까운 위치 클릭');
        setTargetPosition(null);
        return;
      }
    }

    // 새 목표 지점 설정
    setTargetPosition({ x: clickX, y: clickY });
    console.log(`🎯 새 목표 지점 설정: (${clickX}, ${clickY})`);

    // 로봇 이동 명령 전송
    await moveRobotToPosition(selectedRobot, clickX, clickY);
  };

  // 로봇 이동 함수
  const moveRobotToPosition = async (robotId: string, pixelX: number, pixelY: number) => {
    try {
      console.log(`🤖 로봇 ${robotId} 이동 명령 전송 - 목표: (${pixelX}, ${pixelY})`);
      
      const response = await robotService.moveRobotTo(robotId, {
        x: pixelX,
        y: pixelY,
        z: 0
      });

      if (response.status === 'SUCCESS') {
        console.log('✅ 로봇 이동 명령 전송 완료');
        
        // 로봇 상태 업데이트 (이동 중으로 표시)
        setRobots(prevRobots => 
          prevRobots.map(robot => 
            robot.id === robotId 
              ? { ...robot, state: '이동중', lastUpdate: new Date() }
              : robot
          )
        );
      } else {
        console.error('로봇 이동 명령 실패:', response);
        alert('로봇 이동 명령 전송에 실패했습니다.');
      }
    } catch (error) {
      console.error('로봇 이동 오류:', error);
      alert(`로봇 이동 중 오류가 발생했습니다: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  // 로봇 선택 변경 핸들러
  const handleRobotSelect = (robotId: string) => {
    if (isManualMode) {
      setSelectedRobot(robotId);
      setTargetPosition(null); // 로봇 변경 시 목표 지점 초기화
      console.log(`🎯 선택된 로봇 변경: ${robotId}`);
    }
  };

  const initializeMap = async () => {
    try {
      setIsLoading(true);
      
      console.log("맵 초기화 시작...");
      
      // S3 파일 존재 여부 테스트 (임시)
      console.log("🔍 S3 파일 존재 여부 먼저 확인...");
      await mapService.testS3Files();
      
      // 실제 YAML 파일 다운로드 및 파싱
      console.log("YAML 파일 다운로드 중...");
      const yamlContent = await mapService.downloadYamlFile();
      console.log("YAML 내용:", yamlContent);
      
      const mapConfigFromYaml = parseYamlConfig(yamlContent);
      console.log("파싱된 맵 설정:", mapConfigFromYaml);
      
      // PGM 파일에서 실제 크기 정보를 가져와서 설정
      // 임시로 기본값 설정 (나중에 PGM 헤더에서 추출)
      if (!mapConfigFromYaml.width || !mapConfigFromYaml.height) {
        mapConfigFromYaml.width = 2062; // 백엔드 application.yml과 동기화된 값
        mapConfigFromYaml.height = 893;  // 백엔드 application.yml과 동기화된 값
        console.log("🔧 맵 크기 정보 추가:", { width: mapConfigFromYaml.width, height: mapConfigFromYaml.height });
      }
      
      // PGM 파일 URL 가져오기 (백엔드 S3 방식)
      console.log("PGM 파일 URL 가져오는 중...");
      const pgmImageUrl = await mapService.getPgmImageUrl();
      console.log("PGM 이미지 URL:", pgmImageUrl);
      
      // PGM 파일이 Blob URL인 경우 blob 정보도 함께 저장
      let pgmBlob = undefined;
      if (pgmImageUrl.startsWith('blob:')) {
        try {
          const response = await fetch(pgmImageUrl);
          pgmBlob = await response.blob();
          console.log("PGM Blob 정보 저장 완료");
        } catch (error) {
          console.warn("PGM Blob 정보 저장 실패:", error);
        }
      }
      
      const pgmInfo: PGMFileInfo = {
        fileName: 'map.pgm',
        url: pgmImageUrl,
        configFile: 'map.yaml',
        isDirectUrl: true,
        blob: pgmBlob
      };
      
      setMapConfig(mapConfigFromYaml);
      setPgmFileInfo(pgmInfo);
      
      // 실제 로봇 데이터 로드
      await loadRobotData(mapConfigFromYaml);
      
    } catch (error) {
      console.error("❌ API 연결 실패, 상세 오류:", error);
      console.error("❌ 오류 메시지:", error instanceof Error ? error.message : 'Unknown error');
      console.error("❌ 오류 스택:", error instanceof Error ? error.stack : 'No stack trace');
      
      // S3 파일 문제인지 확인
      const errorMessage = error instanceof Error ? error.message : '';
      if (errorMessage.includes('S3') || errorMessage.includes('파일 다운로드에 실패')) {
        console.warn("🔍 S3 파일 문제로 추정됩니다.");
        console.warn("📋 백엔드 개발자에게 확인 요청:");
        console.warn("   1. S3 버킷에 'maps/map.pgm', 'maps/map.yaml' 파일 존재 여부");
        console.warn("   2. 백엔드 서버 로그에서 실제 S3 오류 메시지 확인");
        console.warn("   3. AWS S3 권한 설정 확인");
      }
      
      console.warn("⚠️ 폴백 모드로 전환합니다...");
      
      // 에러 발생 시 기본값으로 폴백 (사용자에게는 정상 작동처럼 보임)
      await initializeFallbackMap();
    } finally {
      setIsLoading(false);
    }
  };

  const initializeFallbackMap = async () => {
    console.log("🔄 폴백 맵 초기화 중...");
    
    // 에러 시 기본 설정으로 폴백
    const fallbackConfig: MapConfig = {
      resolution: 0.05,
      origin: { x: -10.0, y: -10.0, yaw: 0 },
      width: 400,
      height: 400
    };
    
    const fallbackPgmInfo: PGMFileInfo = {
      fileName: 'fallback_map.pgm',
      url: '/maps/sample_map.pgm', // 이 파일은 존재하지 않음 (의도적)
      configFile: '/maps/sample_map.yaml'
    };
    
    console.log("⚠️ 폴백 모드 활성화:");
    console.log("   - 실제 S3 맵 파일 대신 그리드 패턴 표시");
    console.log("   - 백엔드 S3 파일 준비 후 새로고침 필요");
    
    setMapConfig(fallbackConfig);
    setPgmFileInfo(fallbackPgmInfo);
    
    // 기본 로봇 데이터 사용
    const robotsWithScreenCoords = defaultRobots.map(robot => {
      const screenCoords = realToScreen(robot.realX, robot.realY, fallbackConfig);
      return {
        ...robot,
        screenX: screenCoords.x,
        screenY: screenCoords.y
      };
    });
    
    setRobots(robotsWithScreenCoords);
  };

  const loadRobotData = async (mapConfig: MapConfig) => {
    try {
      const robotResponse = await robotService.getRobots();
      
      if (robotResponse.status === 'SUCCESS' && robotResponse.data) {
        const apiRobots = robotResponse.data;
        
        // API 데이터를 화면 표시용 형식으로 변환 (tb1, tb2 필터링)
        const tbRobots = apiRobots.filter((robot: ApiRobot) => 
          robot.robotId === 'tb1' || robot.robotId === 'tb2'
        );
        
        const displayRobots: Robot[] = tbRobots.map((apiRobot: ApiRobot) => {
          // 안전한 위치 데이터 접근
          const position = apiRobot.currentStatus?.position;
          const realX = position?.x ?? 0; // 기본값 0
          const realY = position?.y ?? 0; // 기본값 0
          
          console.log(`🤖 ${apiRobot.robotId} 위치:`, { realX, realY, position });
          
          const screenCoords = realToScreen(realX, realY, mapConfig);
          
          return {
            id: apiRobot.robotId, // tb1 또는 tb2
            name: apiRobot.robotName || (apiRobot.robotId === 'tb1' ? 'Robot1' : 'Robot2'),
            realX,
            realY,
            screenX: screenCoords.x,
            screenY: screenCoords.y,
            battery: apiRobot.currentStatus?.batteryLevel ?? 0,
            state: apiRobot.currentStatus?.state ?? '연결 대기중',
            isOnline: false, // WebSocket에서 업데이트됨
            isActive: apiRobot.isActive,
            isUpdating: false,
            lastUpdate: new Date(apiRobot.currentStatus?.lastUpdated || Date.now())
          };
        });
        
        // tb1, tb2가 API에 없는 경우 기본 데이터로 보완
        const existingIds = displayRobots.map(r => r.id);
        const missingRobots = defaultRobots.filter(r => !existingIds.includes(r.id));
        const finalRobots = [...displayRobots, ...missingRobots];
        
        setRobots(finalRobots);
      } else {
        throw new Error('로봇 데이터 로드 실패');
      }
    } catch (error) {
      console.error('Failed to load robot data:', error);
      // 로봇 데이터 로드 실패 시 기본 데이터 사용
      const robotsWithScreenCoords = defaultRobots.map(robot => {
        const screenCoords = realToScreen(robot.realX, robot.realY, mapConfig);
        return {
          ...robot,
          screenX: screenCoords.x,
          screenY: screenCoords.y
        };
      });
      
      setRobots(robotsWithScreenCoords);
    }
  };

  const updateRobotData = (robotId: string, newData: Partial<Robot>) => {
    setRobots(prevRobots => 
      prevRobots.map(robot => {
        if (robot.id === robotId) {
          const updatedRobot = { ...robot, ...newData };
          // 실제 좌표가 변경된 경우 화면 좌표도 재계산
          if (newData.realX !== undefined || newData.realY !== undefined) {
            const screenCoords = realToScreen(
              updatedRobot.realX, 
              updatedRobot.realY, 
              mapConfig!
            );
            updatedRobot.screenX = screenCoords.x;
            updatedRobot.screenY = screenCoords.y;
          }
          return updatedRobot;
        }
        return robot;
      })
    );
  };

  // 지도 크기 변경 시 모든 로봇 좌표 재계산
  const recalculateAllRobotCoordinates = () => {
    if (!mapConfig) return;
    
    setRobots(prevRobots => 
      prevRobots.map(robot => {
        // ROS 실제 좌표가 있는 경우 realToScreen 사용
        if (robot.realX !== undefined && robot.realY !== undefined) {
          const screenCoords = realToScreen(robot.realX, robot.realY, mapConfig);
          return {
            ...robot,
            screenX: screenCoords.x,
            screenY: screenCoords.y
          };
        }
        return robot;
      })
    );
    
    console.log('🔄 지도 크기 변경으로 인한 모든 로봇 좌표 재계산 완료');
  };

  // 실제 화면 렌더링 크기 기반 좌표 변환 (유동적 지도 크기 대응)
  const transformPixelCoordinates = (backendPixelX: number, backendPixelY: number) => {
    if (!mapConfig) {
      console.warn('MapConfig가 없어 기본 좌표 반환');
      return { transformedX: backendPixelX, transformedY: backendPixelY };
    }

    // 실제 화면에 렌더링되는 지도 크기 동적 감지
    const mapCanvas = document.getElementById('map-canvas') as HTMLCanvasElement;
    let actualDisplayWidth = mapConfig.width;
    let actualDisplayHeight = mapConfig.height;
    
    if (mapCanvas) {
      // Canvas의 실제 화면 표시 크기 (CSS로 조정된 크기)
      const rect = mapCanvas.getBoundingClientRect();
      actualDisplayWidth = rect.width;
      actualDisplayHeight = rect.height;
    }
    
    // 백엔드 원본 지도 크기 (PGM 파일의 실제 크기)
    const BACKEND_MAP_SIZE = {
      width: 2062,
      height: 893
    };
    
    // 백엔드 픽셀 좌표를 실제 화면 표시 크기로 스케일링
    const scaleX = actualDisplayWidth / BACKEND_MAP_SIZE.width;
    const scaleY = actualDisplayHeight / BACKEND_MAP_SIZE.height;
    
    const finalX = backendPixelX * scaleX;
    const finalY = backendPixelY * scaleY;
    
    return { transformedX: finalX, transformedY: finalY };
  };

  // 침입자 감지 이벤트 핸들러
  const handleIntruderDetection = (eventData: any) => {
    try {
      console.log('🚨 침입자 감지 이벤트 처리:', eventData);
      
      // 이벤트 데이터에서 로봇 ID 추출
      const robotId = eventData.robotId || eventData.robot_id;
      
      if (robotId) {
        // 해당 로봇을 침입자 감지 상태로 변경
        setRobots(prevRobots => 
          prevRobots.map(robot => {
            if (robot.id === robotId) {
              console.log(`🔴 ${robotId} 로봇을 침입자 감지 상태로 변경`);
              return {
                ...robot,
                isIntruderDetected: true,
                intruderDetectedAt: new Date(),
                state: '침입자 감지',
                lastUpdate: new Date()
              };
            }
            return robot;
          })
        );
        
        // 5초 후에 침입자 감지 상태 해제 (선택사항)
        setTimeout(() => {
          setRobots(prevRobots => 
            prevRobots.map(robot => {
              if (robot.id === robotId && robot.isIntruderDetected) {
                console.log(`🟢 ${robotId} 로봇 침입자 감지 상태 해제`);
                return {
                  ...robot,
                  isIntruderDetected: false,
                  intruderDetectedAt: undefined,
                  state: '순찰중',
                  lastUpdate: new Date()
                };
              }
              return robot;
            })
          );
        }, 10000); // 10초 후 해제
      }
    } catch (error) {
      console.error('침입자 감지 이벤트 처리 오류:', error);
    }
  };

  // WebSocket 실시간 위치 업데이트 핸들러
  const handleRobotPositionUpdate = (positionUpdate: RobotPositionUpdate) => {
    console.log('🤖 Real-time position update:', positionUpdate);
    
    const { robotId, position, batteryLevel, state, timestamp } = positionUpdate;
    
    // 위치 데이터 상세 로그
    console.log(`📍 ${robotId} 위치 데이터:`, {
      realCoords: { x: position.x, y: position.y, z: position.z },
      pixelCoords: { pixel_x: position.pixel_x, pixel_y: position.pixel_y },
      battery: batteryLevel,
      state: state
    });
    
    // ROS 좌표가 (0,0)인 경우 특별 처리 (초기 위치 또는 연결 끊김)
    if (position.x === 0 && position.y === 0) {
      console.warn(`⚠️ ${robotId} ROS 좌표가 (0,0)입니다 - 초기화 상태이거나 연결이 끊어진 상태일 수 있습니다`);
    }
    
    
    // 로봇 데이터 업데이트
    setRobots(prevRobots => 
      prevRobots.map(robot => {
        if (robot.id === robotId) {
          // 백엔드 픽셀 좌표를 화면 표시용으로 변환
          let finalScreenX = robot.screenX;
          let finalScreenY = robot.screenY;
          
          
          if (position.pixel_x !== undefined && position.pixel_y !== undefined) {
            const transformed = transformPixelCoordinates(position.pixel_x, position.pixel_y);
            finalScreenX = transformed.transformedX;
            finalScreenY = transformed.transformedY;
          }
          
          // 위치가 실제로 변경되었는지 확인
          const hasPositionChanged = 
            Math.abs(finalScreenX - robot.screenX) > 1 || 
            Math.abs(finalScreenY - robot.screenY) > 1;
          
          const updatedRobot = { 
            ...robot,
            realX: position.x || robot.realX,
            realY: position.y || robot.realY,
            screenX: finalScreenX,
            screenY: finalScreenY,
            battery: batteryLevel !== undefined ? batteryLevel : robot.battery,
            state: state || robot.state,
            isOnline: true,
            isUpdating: hasPositionChanged, // 위치 변경 시 애니메이션 활성화
            lastUpdate: new Date(timestamp || Date.now())
          };
          
          console.log(`✅ ${robotId} 업데이트 완료:`, {
            이전위치: { screenX: robot.screenX, screenY: robot.screenY },
            새위치: { screenX: updatedRobot.screenX, screenY: updatedRobot.screenY },
            위치변경: hasPositionChanged,
            배터리: updatedRobot.battery,
            상태: updatedRobot.state,
            백엔드픽셀: { x: position.pixel_x, y: position.pixel_y },
            화면표시좌표: { x: finalScreenX.toFixed(1), y: finalScreenY.toFixed(1) }
          });
          
          
          // 500ms 후에 업데이트 애니메이션 제거
          if (hasPositionChanged) {
            setTimeout(() => {
              setRobots(currentRobots => 
                currentRobots.map(r => 
                  r.id === robotId ? { ...r, isUpdating: false } : r
                )
              );
            }, 500);
          }
          
          return updatedRobot;
        }
        return robot;
      })
    );
  };

  // WebSocket 연결 초기화
  const initializeWebSocket = async () => {
    if (wsInitialized.current) return;
    
    try {
      setWsError(null);
      console.log('Initializing WebSocket connection...');
      
      const callbacks: WebSocketCallbacks = {
        onRobotPositionUpdate: handleRobotPositionUpdate,
        onEventAlert: (data) => {
          console.log('Event alert received:', data);
          handleIntruderDetection(data);
        },
        onCommandResult: (data) => {
          console.log('Command result received:', data);
          // 명령 결과 처리 로직 추가 가능
        },
        onConnect: () => {
          console.log('WebSocket connected successfully');
          setWsConnected(true);
          setWsError(null);
        },
        onDisconnect: () => {
          console.log('WebSocket disconnected');
          setWsConnected(false);
        },
        onError: (error) => {
          console.error('WebSocket error:', error);
          setWsError('WebSocket 연결 오류가 발생했습니다.');
          setWsConnected(false);
        }
      };
      
      await websocketService.connect(callbacks);
      wsInitialized.current = true;
      
    } catch (error) {
      console.error('Failed to initialize WebSocket:', error);
      setWsError('WebSocket 연결에 실패했습니다.');
      setWsConnected(false);
    }
  };

  // 컴포넌트 마운트 시 초기화
  useEffect(() => {
    console.log('🚀 PatrolModePage 초기화 시작');
    initializeMap();
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // WebSocket 연결 초기화 (맵 로드 후)
  useEffect(() => {
    if (mapConfig && !wsInitialized.current) {
      initializeWebSocket();
    }
  }, [mapConfig]);

  // 지도 설정 변경 시 로봇 좌표 재계산
  useEffect(() => {
    if (mapConfig && robots.length > 0) {
      recalculateAllRobotCoordinates();
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mapConfig?.width, mapConfig?.height, mapConfig?.resolution]);

  // 화면 크기 변경 감지 및 좌표 재계산
  useEffect(() => {
    const handleResize = () => {
      console.log('🖥️ 화면 크기 변경 감지 - 로봇 좌표 재계산 예약');
      // 약간의 딜레이를 두어 DOM 업데이트 완료 후 재계산
      setTimeout(() => {
        if (mapConfig && robots.length > 0) {
          console.log('🔄 화면 크기 변경으로 인한 좌표 재계산 실행');
          recalculateAllRobotCoordinates();
        }
      }, 100);
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [mapConfig, robots]);


  // 컴포넌트 언마운트 시 WebSocket 연결 해제
  useEffect(() => {
    return () => {
      if (wsInitialized.current) {
        websocketService.disconnect();
        wsInitialized.current = false;
      }
    };
  }, []);

  // 로딩 상태 처리
  if (isLoading) {
    return (
      <div className="patrol-mode-container">
        <Header />
        <main className="patrol-mode-content">
          <div className="loading-container">
            <h2>지도를 로딩 중입니다...</h2>
            <p>PGM 파일과 설정을 불러오는 중입니다.</p>
          </div>
        </main>
      </div>
    );
  }

  // 지도 설정이 없는 경우 처리
  if (!mapConfig || !pgmFileInfo) {
    return (
      <div className="patrol-mode-container">
        <Header />
        <main className="patrol-mode-content">
          <div className="error-container">
            <h2>지도 로드 실패</h2>
            <p>지도 설정 또는 PGM 파일을 불러올 수 없습니다.</p>
            <button onClick={initializeMap}>다시 시도</button>
          </div>
        </main>
      </div>
    );
  }

  // 화면 좌표로 변환된 순찰 경로
  const screenPatrolRoute = samplePatrolRoute.map(point => {
    const screenCoords = realToScreen(point.realX, point.realY, mapConfig);
    return { x: screenCoords.x, y: screenCoords.y };
  });

  return (
    <div className="patrol-mode-container">
      <Header />
      
      <main className="patrol-mode-content">
        <div className="patrol-header">
          <h1 className="patrol-title">순찰 모드</h1>
          <div className="map-info">
            <span>지도: {pgmFileInfo.fileName}</span>
            <span>해상도: {mapConfig.resolution}m/pixel</span>
            <div className="websocket-status">
              <span className={`status-indicator ${wsConnected ? 'connected' : 'disconnected'}`}>
                {wsConnected ? '🟢 실시간 연결됨' : '🔴 연결 끊김'}
              </span>
              {wsError && <span className="error-message">{wsError}</span>}
            </div>
          </div>
        </div>

        <div className="patrol-layout">
          {/* 지도 및 명령 이력 영역 */}
          <div className="map-section">
            <div className="map-container">
              <PatrolMap 
                robots={robots} 
                patrolRoute={screenPatrolRoute}
                mapConfig={mapConfig}
                pgmFileInfo={pgmFileInfo}
                onRobotUpdate={updateRobotData}
                isManualMode={isManualMode}
                selectedRobot={selectedRobot}
                targetPosition={targetPosition}
                onMapClick={handleMapClick}
                onRobotSelect={handleRobotSelect}
              />
              {/* 컨트롤 버튼들을 맵 위에 오버레이 */}
              <div className="control-overlay">
                <ControlButtons
                  onStartPatrol={handleStartPatrol}
                  onManualControl={handleManualControl}
                  isPatrolActive={isPatrolActive}
                  isManualMode={isManualMode}
                />
              </div>
            </div>
            
            {/* 명령 실행 이력 - 맵 아래로 이동 */}
            <div className="command-log-section">
              <CommandLogViewer maxItems={5} />
            </div>
          </div>

          {/* 로봇 상태 영역 */}
          <div className="robot-status-section">
            <h2 className="status-section-title">로봇 상태</h2>
            <div className="robot-cards">
              {robots.map((robot) => (
                <RobotStatusCard
                  key={robot.id}
                  robotId={robot.id}
                  robotName={robot.name}
                  battery={robot.battery}
                  state={robot.state}
                  isOnline={robot.isOnline}
                  isActive={robot.isActive}
                  lastUpdate={robot.lastUpdate}
                />
              ))}
            </div>
          </div>
        </div>
      </main>
    </div>
  );
};

export default PatrolModePage;