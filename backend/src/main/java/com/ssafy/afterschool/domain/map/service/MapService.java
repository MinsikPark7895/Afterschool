package com.ssafy.afterschool.domain.map.service;

/**
 * 맵 서비스 인터페이스
 * - 맵 파일 다운로드
 * - 좌표 변환 (ROS ↔ 픽셀)
 */
public interface MapService {

    /**
     * 맵 파일 다운로드
     * @param fileType 파일 타입 (pgm, yaml)
     * @return 파일 바이트 배열
     */
    byte[] downloadMapFile(String fileType);

    /**
     * 맵 파일명 반환
     * @param fileType 파일 타입
     * @return 파일명
     */
    String getMapFileName(String fileType);

    /**
     * ROS 좌표를 픽셀 좌표로 변환
     * @param rosX ROS X 좌표 (미터)
     * @param rosY ROS Y 좌표 (미터)
     * @return 픽셀 좌표 [x, y]
     */
    int[] convertRosToPixel(double rosX, double rosY);

    /**
     * 픽셀 좌표를 ROS 좌표로 변환
     * @param pixelX 픽셀 X 좌표
     * @param pixelY 픽셀 Y 좌표
     * @return ROS 좌표 [x, y]
     */
    double[] convertPixelToRos(int pixelX, int pixelY);

    /**
     * 현재 맵 ID 반환
     * @return 맵 ID
     */
    String getCurrentMapId();

    /**
     * 맵 좌표계 정보 반환
     * @return 좌표계 정보 (해상도, 크기, 원점)
     */
    MapCoordinateInfo getMapCoordinateInfo();

    /**
     * 맵 좌표계 정보 DTO
     */
    record MapCoordinateInfo(
            double resolution,    // 미터/픽셀
            int width,           // 픽셀
            int height,          // 픽셀
            double originX,      // 원점 X (미터)
            double originY       // 원점 Y (미터)
    ) {}
}
