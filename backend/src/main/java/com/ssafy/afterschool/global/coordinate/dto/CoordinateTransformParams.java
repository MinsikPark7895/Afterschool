package com.ssafy.afterschool.global.coordinate.dto;

/**
 * 좌표 변환에 필요한 파라미터들을 담는 DTO
 */
public record CoordinateTransformParams(
        double resolution,    // 해상도 (미터/픽셀)
        double originX,       // 원점 X 좌표 (미터)
        double originY,       // 원점 Y 좌표 (미터)
        int imageWidth,       // 이미지 너비 (픽셀)
        int imageHeight       // 이미지 높이 (픽셀)
) {
    /**
     * 파라미터 유효성 검증
     */
    public boolean isValid() {
        return resolution > 0 && imageWidth > 0 && imageHeight > 0;
    }
}