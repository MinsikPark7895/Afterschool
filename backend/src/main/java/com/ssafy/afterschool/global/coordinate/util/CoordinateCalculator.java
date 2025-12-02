package com.ssafy.afterschool.global.coordinate.util;

import com.ssafy.afterschool.global.coordinate.dto.CoordinateTransformParams;
import com.ssafy.afterschool.global.coordinate.dto.Position;
import lombok.extern.slf4j.Slf4j;

/**
 * 좌표 변환 계산을 담당하는 순수 함수 유틸리티 클래스
 */
@Slf4j
public final class CoordinateCalculator {

    private CoordinateCalculator() {
        // Utility class - 인스턴스 생성 방지
    }

    /**
     * ROS 좌표를 픽셀 좌표로 변환
     */
    public static Position rosToPixel(Position rosPosition, CoordinateTransformParams params) {
        if (!params.isValid()) {
            throw new IllegalArgumentException("잘못된 좌표 변환 파라미터: " + params);
        }

        log.debug("🔄 ROS → 픽셀 좌표 변환: ros({}, {}, {})",
                rosPosition.getX(), rosPosition.getY(), rosPosition.getZ());

        int pixelX = (int) ((rosPosition.getX() - params.originX()) / params.resolution());
        int pixelY = (int) (params.imageHeight() - (rosPosition.getY() - params.originY()) / params.resolution());

        log.debug("✅ 좌표 변환 완료: ros({}, {}) → pixel({}, {})",
                rosPosition.getX(), rosPosition.getY(), pixelX, pixelY);

        return Position.builder()
                .x(pixelX)
                .y(pixelY)
                .z(rosPosition.getZ())
                .build();
    }

    /**
     * 픽셀 좌표를 ROS 좌표로 변환
     */
    public static Position pixelToRos(Position pixelPosition, CoordinateTransformParams params) {
        if (!params.isValid()) {
            throw new IllegalArgumentException("잘못된 좌표 변환 파라미터: " + params);
        }

        log.debug("🔄 픽셀 → ROS 좌표 변환: pixel({}, {}, {})",
                pixelPosition.getX(), pixelPosition.getY(), pixelPosition.getZ());

        double rosX = pixelPosition.getX() * params.resolution() + params.originX();
        double rosY = (params.imageHeight() - pixelPosition.getY()) * params.resolution() + params.originY();

        log.debug("✅ 좌표 변환 완료: pixel({}, {}) → ros({}, {})",
                pixelPosition.getX(), pixelPosition.getY(), rosX, rosY);

        return Position.builder()
                .x(rosX)
                .y(rosY)
                .z(pixelPosition.getZ())
                .build();
    }

    /**
     * 거리 변환: 픽셀 거리를 미터 거리로 변환
     */
    public static double pixelDistanceToMeter(double pixelDistance, double resolution) {
        return pixelDistance * resolution;
    }

    /**
     * 거리 변환: 미터 거리를 픽셀 거리로 변환
     */
    public static double meterDistanceToPixel(double meterDistance, double resolution) {
        return meterDistance / resolution;
    }

    /**
     * 좌표가 이미지 영역 내에 있는지 검증
     */
    public static boolean isPixelInBounds(Position position, CoordinateTransformParams params) {
        return isPixelInBounds((int) position.getX(), (int) position.getY(), params);
    }

    /**
     * 좌표가 이미지 영역 내에 있는지 검증
     */
    public static boolean isPixelInBounds(int pixelX, int pixelY, CoordinateTransformParams params) {
        return pixelX >= 0 && pixelX < params.imageWidth()
            && pixelY >= 0 && pixelY < params.imageHeight();
    }
}