import React, { useEffect, useRef } from "react";
import "./PatrolMap.css";
import { MapConfig, PGMFileInfo } from "../../types/map";

// 로봇 데이터 인터페이스 (실제 MQTT 데이터 구조 기반)
interface Robot {
  id: string;
  name: string;
  realX: number;
  realY: number;
  screenX: number;
  screenY: number;
  battery: number;
  state: string; // mission 대신 state 사용
  isOnline: boolean;
  isActive: boolean; // 로봇 활성화 상태 추가
  lastUpdate: Date;
  isUpdating?: boolean; // 위치 업데이트 애니메이션용
  isIntruderDetected?: boolean; // 침입자 감지 상태
  intruderDetectedAt?: Date; // 침입자 감지 시간
}

interface PatrolMapProps {
  robots: Robot[];
  patrolRoute?: { x: number; y: number }[];
  mapConfig: MapConfig;
  pgmFileInfo: PGMFileInfo;
  onRobotUpdate: (robotId: string, newData: Partial<Robot>) => void;
  isManualMode?: boolean;
  selectedRobot?: string | null;
  targetPosition?: { x: number; y: number } | null;
  onMapClick?: (event: React.MouseEvent<HTMLDivElement>) => void;
  onRobotSelect?: (robotId: string) => void;
}

const PatrolMap: React.FC<PatrolMapProps> = ({
  robots,
  patrolRoute = [],
  mapConfig,
  pgmFileInfo,
  onRobotUpdate,
  isManualMode = false,
  selectedRobot = null,
  targetPosition = null,
  onMapClick,
  onRobotSelect,
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // PatrolMap에서 직접 좌표 변환 처리
  const transformRobotPosition = (robot: Robot) => {
    // 백엔드 원본 지도 크기 (application.yml과 동기화)
    const BACKEND_MAP_SIZE = {
      width: 2062,
      height: 893
    };

    // 현재 Canvas의 실제 화면 표시 크기 감지
    const canvas = canvasRef.current;
    if (!canvas) {
      console.warn(`⚠️ [PatrolMap] Canvas ref가 없어 기본 좌표 사용: ${robot.id}`);
      return { x: robot.screenX, y: robot.screenY };
    }

    const rect = canvas.getBoundingClientRect();
    const actualDisplayWidth = rect.width;
    const actualDisplayHeight = rect.height;

    // 스케일 계산
    const scaleX = actualDisplayWidth / BACKEND_MAP_SIZE.width;
    const scaleY = actualDisplayHeight / BACKEND_MAP_SIZE.height;

    // 최종 좌표 계산 (robot.screenX, screenY는 백엔드 픽셀 좌표로 가정)
    const transformedX = robot.screenX * scaleX + actualDisplayWidth / 4;
    const transformedY = robot.screenY * scaleY;

    console.log(`🎯 [PatrolMap-${robot.id}] 좌표 변환:`, {
      '백엔드 픽셀': `(${robot.screenX}, ${robot.screenY})`,
      'Canvas 크기': `${canvas.width}x${canvas.height}`,
      '실제 표시': `${actualDisplayWidth.toFixed(1)}x${actualDisplayHeight.toFixed(1)}`,
      '스케일': `X=${scaleX.toFixed(3)}, Y=${scaleY.toFixed(3)}`,
      '변환 결과': `(${transformedX.toFixed(1)}, ${transformedY.toFixed(1)})`
    });

    return { x: transformedX, y: transformedY };
  };

  // 브라우저 호환성 체크 함수
  const checkBrowserCompatibility = () => {
    const issues = [];

    // Canvas 지원 체크
    const canvas = document.createElement("canvas");
    if (!canvas.getContext || !canvas.getContext("2d")) {
      issues.push("Canvas 2D 컨텍스트 미지원");
    }

    // Blob 지원 체크
    if (typeof Blob === "undefined") {
      issues.push("Blob API 미지원");
    }

    // ArrayBuffer 지원 체크
    if (typeof ArrayBuffer === "undefined") {
      issues.push("ArrayBuffer API 미지원");
    }

    // TextDecoder 지원 체크
    if (typeof TextDecoder === "undefined") {
      issues.push("TextDecoder API 미지원");
    }

    console.log("🔍 브라우저 호환성 체크 결과:", {
      호환성문제: issues.length > 0 ? issues : "모든 기능 지원됨",
      브라우저: navigator.userAgent,
      언어: navigator.language,
      온라인상태: navigator.onLine,
      쿠키활성화: navigator.cookieEnabled,
    });

    return issues;
  };

  // PGM 파일을 직접 처리하는 함수 (실제 PGM 데이터 렌더링)
  const processPGMFile = async (blob: Blob, ctx: CanvasRenderingContext2D) => {
    try {
      console.log("🎯 PGM 파일 직접 처리 시작...");
      console.log("📦 Blob 크기:", blob.size, "바이트");

      const arrayBuffer = await blob.arrayBuffer();
      const uint8Array = new Uint8Array(arrayBuffer);
      console.log("🔢 ArrayBuffer 변환 완료:", uint8Array.length, "바이트");

      // PGM 파일 헤더 파싱
      const text = new TextDecoder().decode(uint8Array.slice(0, 200));
      console.log("📄 PGM 헤더 (처음 100자):", text.substring(0, 100));

      // PGM 헤더 파싱 (P5 형식)
      const lines = text.split("\n");
      let headerEndIndex = 0;
      let width = 0,
        height = 0,
        maxVal = 0;
      let lineIndex = 0;

      // P5 매직 넘버 확인
      if (lines[0].trim() !== "P5") {
        throw new Error("지원하지 않는 PGM 형식");
      }
      lineIndex++;

      // 주석 건너뛰기
      while (lineIndex < lines.length && lines[lineIndex].startsWith("#")) {
        lineIndex++;
      }

      // 너비, 높이 파싱
      const dimensions = lines[lineIndex].trim().split(/\s+/);
      width = parseInt(dimensions[0]);
      height = parseInt(dimensions[1]);
      lineIndex++;

      // 최대값 파싱
      maxVal = parseInt(lines[lineIndex].trim());
      lineIndex++;

      // 헤더 끝 위치 찾기
      const headerText = lines.slice(0, lineIndex).join("\n") + "\n";
      headerEndIndex = new TextEncoder().encode(headerText).length;

      console.log(`📐 PGM 크기: ${width}x${height}, 최대값: ${maxVal}`);

      // 이미지 데이터 추출
      const imageData = uint8Array.slice(headerEndIndex);

      if (imageData.length < width * height) {
        throw new Error("PGM 데이터 크기 불일치");
      }

      // Canvas ImageData 생성
      const canvasImageData = ctx.createImageData(width, height);
      const data = canvasImageData.data;

      // PGM 데이터를 RGBA로 변환
      for (let i = 0; i < width * height; i++) {
        const grayValue = imageData[i];
        const pixelIndex = i * 4;

        // 그레이스케일 값을 RGBA로 변환
        // 0 = 검정(장애물), 255 = 흰색(자유공간), 그 사이는 미지영역
        data[pixelIndex] = grayValue; // R
        data[pixelIndex + 1] = grayValue; // G
        data[pixelIndex + 2] = grayValue; // B
        data[pixelIndex + 3] = 255; // A (불투명)
      }

      // Canvas 크기 조정 후 이미지 그리기 (스케일링 정보는 drawImage에서 직접 계산)

      // 임시 캔버스에 원본 크기로 그리기
      const tempCanvas = document.createElement("canvas");
      tempCanvas.width = width;
      tempCanvas.height = height;
      const tempCtx = tempCanvas.getContext("2d")!;
      tempCtx.putImageData(canvasImageData, 0, 0);

      // 메인 캔버스에 스케일링해서 그리기
      ctx.clearRect(0, 0, mapConfig.width, mapConfig.height);
      ctx.drawImage(
        tempCanvas,
        0,
        0,
        width,
        height,
        0,
        0,
        mapConfig.width,
        mapConfig.height
      );

      console.log("✅ PGM 파일 실제 렌더링 완료");
    } catch (error) {
      console.error("❌ PGM 파일 처리 실패:", error);
      console.error("🔍 에러 상세 정보:", {
        name: error instanceof Error ? error.name : "Unknown",
        message: error instanceof Error ? error.message : "Unknown error",
        stack: error instanceof Error ? error.stack : "No stack trace",
        canvasSize: { width: mapConfig.width, height: mapConfig.height },
      });

      // 폴백: 개선된 그리드 패턴으로 표시
      console.log("🛠️ 기본 그리드 맵 표시 시작");

      ctx.fillStyle = "#2a2a2a";
      ctx.fillRect(0, 0, mapConfig.width, mapConfig.height);

      ctx.strokeStyle = "#4a4a4a";
      ctx.lineWidth = 1;
      const gridSize = 50;

      for (let x = 0; x <= mapConfig.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, mapConfig.height);
        ctx.stroke();
      }

      for (let y = 0; y <= mapConfig.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(mapConfig.width, y);
        ctx.stroke();
      }

      // 에러 메시지 표시
      ctx.fillStyle = "#ff6b6b";
      ctx.font = "bold 18px Arial";
      ctx.textAlign = "center";
      ctx.fillText(
        "⚠️ 맵 로딩 실패",
        mapConfig.width / 2,
        mapConfig.height / 2 - 30
      );

      ctx.fillStyle = "#ffffff";
      ctx.font = "14px Arial";
      ctx.fillText(
        "기본 그리드를 표시합니다",
        mapConfig.width / 2,
        mapConfig.height / 2
      );
      ctx.fillText(
        "새로고침(F5) 또는 캐시 삭제를 시도해보세요",
        mapConfig.width / 2,
        mapConfig.height / 2 + 20
      );

      const errorMsg = error instanceof Error ? error.message : "Unknown error";
      ctx.fillStyle = "#cccccc";
      ctx.font = "12px Arial";
      ctx.fillText(
        `오류: ${errorMsg}`,
        mapConfig.width / 2,
        mapConfig.height / 2 + 45
      );

      console.log("✅ 기본 그리드 맵 표시 완료");
    }
  };

  // PGM 파일을 Canvas에 렌더링
  useEffect(() => {
    const loadPGMImage = async () => {
      if (!pgmFileInfo || !pgmFileInfo.url || !canvasRef.current) return;

      try {
        const canvas = canvasRef.current;
        const ctx = canvas.getContext("2d");
        if (!ctx) return;

        console.log("🗺️ PGM 이미지 로딩 시작:", pgmFileInfo.url);
        console.log("📐 현재 지도 크기:", mapConfig.width, "x", mapConfig.height);

        // 브라우저 호환성 체크
        const compatibilityIssues = checkBrowserCompatibility();
        if (compatibilityIssues.length > 0) {
          console.warn("⚠️ 브라우저 호환성 문제 발견:", compatibilityIssues);
        }

        // PGM 파일은 브라우저가 직접 인식하지 못하므로 바로 직접 처리
        if (pgmFileInfo.url.includes("blob:") && pgmFileInfo.blob) {
          console.log("🎯 PGM 파일 직접 처리 시작 (브라우저 호환성 보장)");
          console.log("📁 Blob 정보:", {
            size: pgmFileInfo.blob.size,
            type: pgmFileInfo.blob.type,
            url: pgmFileInfo.url,
          });
          await processPGMFile(pgmFileInfo.blob, ctx);
        } else {
          // Blob이 없는 경우에만 Image 객체로 시도 (다른 이미지 포맷용)
          console.log("📸 일반 이미지로 처리 시도");
          const img = new Image();

          img.onload = () => {
            console.log("이미지 로드 성공:", img.width, "x", img.height);
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // 배경을 흰색으로 설정
            ctx.fillStyle = "#ffffff";
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            // 이미지를 현재 지도 설정 크기에 맞게 그리기
            ctx.drawImage(img, 0, 0, mapConfig.width, mapConfig.height);
            console.log("이미지 렌더링 완료 - 크기:", mapConfig.width, "x", mapConfig.height);
          };

          img.onerror = (error) => {
            console.error("이미지 로드 실패:", error);
            // 에러 시 기본 배경 표시
            ctx.fillStyle = "#f0f0f0";
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = "#666";
            ctx.font = "16px Arial";
            ctx.textAlign = "center";
            ctx.fillText(
              "맵 이미지를 불러올 수 없습니다",
              canvas.width / 2,
              canvas.height / 2
            );
            ctx.fillText(
              "S3에서 맵 파일을 확인해주세요",
              canvas.width / 2,
              canvas.height / 2 + 25
            );
          };

          img.crossOrigin = "anonymous";
          img.src = pgmFileInfo.url;
        }
      } catch (error) {
        console.error("Failed to load PGM image:", error);
      }
    };

    loadPGMImage();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pgmFileInfo, mapConfig.width, mapConfig.height]);

  return (
    <div className="patrol-map-container">
      <div className="patrol-map">
        {/* PGM 파일이 있을 경우 Canvas 영역 */}
        {pgmFileInfo && (
          <div 
            className={`pgm-canvas-container ${isManualMode ? 'manual-mode' : ''}`}
            onClick={isManualMode ? onMapClick : undefined}
            style={{ cursor: isManualMode ? 'crosshair' : 'default' }}
          >
            <canvas
              ref={canvasRef}
              id="map-canvas"
              className="pgm-canvas"
              width={mapConfig.width}
              height={mapConfig.height}
            />
            <div className="canvas-overlay">
              {/* 순찰 경로 */}
              {patrolRoute.length > 0 && (
                <svg
                  className="patrol-route"
                  viewBox={`0 0 ${mapConfig.width} ${mapConfig.height}`}
                  preserveAspectRatio="none"
                >
                  <path
                    d={`M ${patrolRoute
                      .map((point) => `${point.x}, ${point.y}`)
                      .join(" L ")}`}
                    stroke="#4A90E2"
                    strokeWidth="2"
                    strokeDasharray="5,5"
                    fill="none"
                  />
                </svg>
              )}
              {/* 로봇 위치 표시 */}
              {robots.map((robot) => {
                // PatrolMap에서 직접 좌표 변환
                const transformedPos = transformRobotPosition(robot);
                
                return (
                  <div
                    key={robot.id}
                    className={`robot-marker ${
                      robot.isIntruderDetected ? "intruder-detected" : 
                      robot.isOnline ? "online" : "offline"
                    } ${robot.isUpdating ? "updating" : ""} ${
                      isManualMode && selectedRobot === robot.id ? "selected" : ""
                    }`}
                    style={{
                      left: `${transformedPos.x}px`,
                      top: `${transformedPos.y}px`,
                      cursor: isManualMode ? 'pointer' : 'default'
                    }}
                    title={`${robot.name} - 배터리: ${robot.battery}% - 상태: ${robot.state}`}
                    onClick={isManualMode && onRobotSelect ? (e) => {
                      e.stopPropagation(); // 맵 클릭 이벤트 방지
                      onRobotSelect(robot.id);
                    } : undefined}
                  >
                    <div className="robot-icon">
                      <div className="robot-center"></div>
                    </div>
                    <div className="robot-label">
                      {robot.name}
                      <div style={{ fontSize: "10px", opacity: 0.8 }}>
                        {robot.battery}% | {robot.state}
                      </div>
                    </div>
                  </div>
                );
              })}
              
              {/* 목표 지점 표시 (수동 조작 모드일 때) */}
              {isManualMode && targetPosition && (
                <div
                  className="target-marker"
                  style={{
                    left: `${targetPosition.x}px`,
                    top: `${targetPosition.y}px`,
                  }}
                >
                  <div className="target-icon">
                    <div className="target-center"></div>
                  </div>
                  <div className="target-label">목표 지점</div>
                </div>
              )}
            </div>
          </div>
        )}

        {/* PGM 파일이 없는 경우 기존 배경 사용 */}
        {!pgmFileInfo && (
          <div className="map-background">
            {/* 순찰 경로 */}
            {patrolRoute.length > 0 && (
              <svg
                className="patrol-route"
                viewBox="0 0 100 100"
                preserveAspectRatio="none"
              >
                <path
                  d={`M ${patrolRoute
                    .map((point) => `${point.x}, ${point.y}`)
                    .join(" L ")}`}
                  stroke="#4A90E2"
                  strokeWidth="0.5"
                  strokeDasharray="2,2"
                  fill="none"
                />
              </svg>
            )}
            {/* 로봇 위치 표시 */}
            {robots.map((robot) => {
              // PGM 파일이 없는 경우에도 좌표 변환 적용 (백분율 대신 픽셀 사용)
              const transformedPos = transformRobotPosition(robot);
              
              return (
                <div
                  key={robot.id}
                  className={`robot-marker ${
                    robot.isIntruderDetected ? "intruder-detected" : 
                    robot.isOnline ? "online" : "offline"
                  } ${robot.isUpdating ? "updating" : ""}`}
                  style={{
                    left: `${transformedPos.x}px`,
                    top: `${transformedPos.y}px`,
                  }}
                  title={`${robot.name} - 배터리: ${robot.battery}% - 상태: ${robot.state}`}
                >
                  <div className="robot-icon">
                    <div className="robot-center"></div>
                  </div>
                  <div className="robot-label">
                    {robot.name}
                    <div style={{ fontSize: "10px", opacity: 0.8 }}>
                      {robot.battery}% | {robot.state}
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        )}
      </div>
    </div>
  );
};

export default PatrolMap;
