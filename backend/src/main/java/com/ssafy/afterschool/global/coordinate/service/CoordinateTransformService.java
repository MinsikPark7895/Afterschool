package com.ssafy.afterschool.global.coordinate.service;

import com.ssafy.afterschool.global.coordinate.dto.CoordinateTransformParams;
import com.ssafy.afterschool.global.coordinate.dto.Position;
import com.ssafy.afterschool.global.coordinate.util.CoordinateCalculator;
import com.ssafy.afterschool.global.properties.domain.MapProperties;
import lombok.RequiredArgsConstructor;
import org.springframework.lang.NonNull;
import org.springframework.stereotype.Service;

/**
 * 좌표 변환 비즈니스 로직을 담당하는 서비스
 * MapProperties에서 변환 파라미터를 가져와서 CoordinateCalculator를 사용합니다.
 */
@Service
@RequiredArgsConstructor
public class CoordinateTransformService {

    private final MapProperties mapProperties;

    /**
     * ROS 좌표를 픽셀 좌표로 변환
     */
    @NonNull
    public Position rosToPixel(@NonNull Position rosPosition) {
        CoordinateTransformParams params = getTransformParams();
        return CoordinateCalculator.rosToPixel(rosPosition, params);
    }

    /**
     * 픽셀 좌표를 ROS 좌표로 변환
     */
    @NonNull
    public Position pixelToRos(@NonNull Position pixelPosition) {
        CoordinateTransformParams params = getTransformParams();
        return CoordinateCalculator.pixelToRos(pixelPosition, params);
    }

    /**
     * 거리 변환: 픽셀 거리를 미터 거리로 변환
     */
    public double pixelDistanceToMeter(double pixelDistance) {
        return CoordinateCalculator.pixelDistanceToMeter(pixelDistance, mapProperties.getResolution());
    }

    /**
     * 거리 변환: 미터 거리를 픽셀 거리로 변환
     */
    public double meterDistanceToPixel(double meterDistance) {
        return CoordinateCalculator.meterDistanceToPixel(meterDistance, mapProperties.getResolution());
    }

    /**
     * 좌표가 이미지 영역 내에 있는지 검증
     */
    public boolean isPixelInBounds(@NonNull Position position) {
        CoordinateTransformParams params = getTransformParams();
        return CoordinateCalculator.isPixelInBounds(position, params);
    }

    /**
     * 좌표가 이미지 영역 내에 있는지 검증
     */
    public boolean isPixelInBounds(int pixelX, int pixelY) {
        CoordinateTransformParams params = getTransformParams();
        return CoordinateCalculator.isPixelInBounds(pixelX, pixelY, params);
    }

    /**
     * 현재 맵 설정을 기반으로 좌표 변환 파라미터 생성
     */
    private CoordinateTransformParams getTransformParams() {
        return new CoordinateTransformParams(
                mapProperties.getResolution(),
                mapProperties.getOriginX(),
                mapProperties.getOriginY(),
                mapProperties.getWidth(),
                mapProperties.getHeight()
        );
    }
}