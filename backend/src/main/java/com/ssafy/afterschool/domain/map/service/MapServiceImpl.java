package com.ssafy.afterschool.domain.map.service;

import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.coordinate.service.CoordinateTransformService;
import com.ssafy.afterschool.global.coordinate.dto.Position;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.properties.domain.MapProperties;
import com.ssafy.afterschool.global.s3.S3Service;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

/**
 * 맵 서비스 구현체
 * - S3에서 맵 파일 다운로드
 * - ROS ↔ 픽셀 좌표 변환
 * - 맵 메타데이터 관리
 */
@Service
@RequiredArgsConstructor
@Slf4j
public class MapServiceImpl implements MapService {

    private final MapProperties mapProperties;
    private final S3Service s3Service;
    private final CoordinateTransformService coordinateTransformService;

    @Override
    public byte[] downloadMapFile(String fileType) {
        log.info("🗺️ 맵 파일 다운로드 시작: fileType={}", fileType);

        // 파일 타입 유효성 검증
        if (!mapProperties.isSupported(fileType)) {
            log.warn("❌ 지원하지 않는 파일 타입: {}", fileType);
            throw new ApiException(ErrorCode.FILE_TYPE_NOT_SUPPORTED, "fileType: " + fileType);
        }

        try {
            // S3 키 생성 및 파일 다운로드
            String s3Key = mapProperties.generateS3Key(fileType);
            byte[] fileData = s3Service.downloadFile(s3Key);
            
            log.info("✅ 맵 파일 다운로드 완료: fileType={}, size={} bytes", fileType, fileData.length);
            return fileData;
            
        } catch (ApiException e) {
            // S3Service에서 발생한 ApiException은 그대로 재던지기
            log.error("❌ 맵 파일 다운로드 실패: fileType={}", fileType, e);
            throw e;
        } catch (Exception e) {
            log.error("❌ 맵 파일 다운로드 중 예상치 못한 오류: fileType={}", fileType, e);
            throw new ApiException(ErrorCode.FILE_DOWNLOAD_FAILED, "fileType: " + fileType, e);
        }
    }

    @Override
    public String getMapFileName(String fileType) {
        log.debug("🗺️ 맵 파일명 조회: fileType={}", fileType);

        if (!mapProperties.isSupported(fileType)) {
            throw new ApiException(ErrorCode.FILE_TYPE_NOT_SUPPORTED, "fileType: " + fileType);
        }

        String fileName = mapProperties.getFileName(fileType);
        log.debug("📄 맵 파일명: fileType={}, fileName={}", fileType, fileName);
        return fileName;
    }

    @Override
    public int[] convertRosToPixel(double rosX, double rosY) {
        log.debug("🔄 ROS → 픽셀 좌표 변환: ros({}, {}) ", rosX, rosY);

        try {
            Position rosPosition = Position.builder()
                    .x(rosX)
                    .y(rosY)
                    .z(0.0)
                    .build();

            Position pixelPosition = coordinateTransformService.rosToPixel(rosPosition);
            int[] pixelCoords = {(int) pixelPosition.getX(), (int) pixelPosition.getY()};

            log.debug("✅ 좌표 변환 완료: ros({}, {}) → pixel({}, {})",
                    rosX, rosY, pixelCoords[0], pixelCoords[1]);
            return pixelCoords;
        } catch (Exception e) {
            log.error("❌ ROS → 픽셀 좌표 변환 실패: ros({}, {})", rosX, rosY, e);
            throw new ApiException(ErrorCode.INTERNAL_SERVER_ERROR, 
                    String.format("좌표 변환 실패: ros(%.2f, %.2f)", rosX, rosY), e);
        }
    }

    @Override
    public double[] convertPixelToRos(int pixelX, int pixelY) {
        log.debug("🔄 픽셀 → ROS 좌표 변환: pixel({}, {})", pixelX, pixelY);

        try {
            Position pixelPosition = Position.builder()
                    .x(pixelX)
                    .y(pixelY)
                    .z(0.0)
                    .build();

            Position rosPosition = coordinateTransformService.pixelToRos(pixelPosition);
            double[] rosCoords = {rosPosition.getX(), rosPosition.getY()};

            log.debug("✅ 좌표 변환 완료: pixel({}, {}) → ros({}, {})",
                    pixelX, pixelY, rosCoords[0], rosCoords[1]);
            return rosCoords;
        } catch (Exception e) {
            log.error("❌ 픽셀 → ROS 좌표 변환 실패: pixel({}, {})", pixelX, pixelY, e);
            throw new ApiException(ErrorCode.INTERNAL_SERVER_ERROR, 
                    String.format("좌표 변환 실패: pixel(%d, %d)", pixelX, pixelY), e);
        }
    }

    @Override
    public String getCurrentMapId() {
        String mapId = mapProperties.getCurrentMapId();
        log.debug("🗺️ 현재 맵 ID: {}", mapId);
        return mapId;
    }

    @Override
    public MapCoordinateInfo getMapCoordinateInfo() {
        log.debug("🗺️ 맵 좌표계 정보 조회");

        MapCoordinateInfo mapInfo = new MapCoordinateInfo(
                mapProperties.getResolution(),
                mapProperties.getWidth(),
                mapProperties.getHeight(),
                mapProperties.getOriginX(),
                mapProperties.getOriginY()
        );

        log.debug("📊 맵 좌표계 정보: resolution={}, size={}x{}, origin=({}, {})",
                mapInfo.resolution(), mapInfo.width(), mapInfo.height(), 
                mapInfo.originX(), mapInfo.originY());

        return mapInfo;
    }
}
