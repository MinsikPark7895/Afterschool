package com.ssafy.afterschool.domain.map.dto;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * YAML 파일에서 파싱된 맵 메타데이터 DTO
 * YAML 파일 예시:
 * image: map.pgm
 * mode: trinary
 * resolution: 0.05
 * origin: [-58.8, -19.2, 0]
 * negate: 0
 * occupied_thresh: 0.65
 * free_thresh: 0.25
 */
public record MapMetadata(
    @JsonProperty("image")
    String image,
    
    @JsonProperty("mode")
    String mode,            // trinary, scale, raw 등
    
    @JsonProperty("resolution") 
    double resolution,
    
    @JsonProperty("origin")
    double[] origin,
    
    @JsonProperty("negate")
    Integer negate,
    
    @JsonProperty("occupied_thresh")
    Double occupiedThresh,
    
    @JsonProperty("free_thresh")
    Double freeThresh
) {
    
    /**
     * 원점 X 좌표 반환
     */
    public double getOriginX() {
        return origin != null && origin.length > 0 ? origin[0] : 0.0;
    }
    
    /**
     * 원점 Y 좌표 반환
     */
    public double getOriginY() {
        return origin != null && origin.length > 1 ? origin[1] : 0.0;
    }
    
    /**
     * 원점 각도 반환 (라디안)
     */
    public double getOriginTheta() {
        return origin != null && origin.length > 2 ? origin[2] : 0.0;
    }
    
    /**
     * 검증 메서드
     */
    public boolean isValid() {
        return image != null && !image.trim().isEmpty() 
            && resolution > 0 
            && origin != null && origin.length >= 2;
    }
}
