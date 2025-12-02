package com.ssafy.afterschool.global.properties.domain;

import com.ssafy.afterschool.domain.map.dto.MapMetadata;
import com.ssafy.afterschool.domain.map.service.MapMetadataLoader;
import com.ssafy.afterschool.global.properties.template.BaseS3Properties;
import lombok.Getter;
import lombok.Setter;
import lombok.extern.slf4j.Slf4j;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

import java.util.Map;

/**
 * 맵 도메인 설정 프로퍼티
 * - S3 파일 경로 관리
 * - 동적 메타데이터 로딩 (YAML 파일에서)
 * - 맵 메타데이터 설정값 제공
 */
@Component
@ConfigurationProperties(prefix = "domain.map")
@Getter
@Setter
@Slf4j
public class MapProperties implements BaseS3Properties {

    /**
     * 현재 활성화된 맵 ID (실제로는 단일 맵이므로 고정값)
     */
    private String currentMapId = "main";

    /**
     * S3 경로 패턴 (실제 구조: maps/파일명)
     */
    private String s3PathPattern = "maps/{filename}";

    /**
     * 실제 이미지 크기 설정 (application.yml에서 주입)
     */
    private int width;
    private int height;

    /**
     * 파일 타입별 실제 파일명 (고정)
     */
    private Map<String, String> fileNames = Map.of(
            "pgm", "map.pgm",         // 실제 파일명
            "yaml", "map.yaml"        // 실제 파일명
    );

    /**
     * MapMetadataLoader 의존성 주입 (MapConfig에서 설정)
     * -- SETTER --
     *  MapConfig에서 의존성 주입을 위한 setter

     */
    private MapMetadataLoader metadataLoader;

    /**
     * 캐시된 메타데이터 저장
     */
    private MapMetadata cachedMetadata;

    /**
     * 동적으로 메타데이터 로딩
     * 캐시가 없으면 S3에서 YAML 파일을 읽어옴
     */
    private MapMetadata getMetadata() {
        if (cachedMetadata == null && metadataLoader != null) {
            try {
                log.debug("🔄 메타데이터 동적 로딩: mapId={}", currentMapId);
                cachedMetadata = metadataLoader.loadMapMetadata(currentMapId);
            } catch (Exception e) {
                log.warn("⚠️ 메타데이터 로딩 실패, fallback 값 사용: mapId={}", currentMapId, e);
                // fallback으로 null 반환하여 기본값 사용
                return null;
            }
        }
        return cachedMetadata;
    }

    /**
     * 해상도 동적 반환 (YAML에서 로딩)
     */
    public double getResolution() {
        MapMetadata metadata = getMetadata();
        return metadata != null ? metadata.resolution() : 0.05; // fallback
    }

    /**
     * 원점 X 좌표 동적 반환 (YAML에서 로딩)
     */
    public double getOriginX() {
        MapMetadata metadata = getMetadata();
        return metadata != null ? metadata.getOriginX() : -49.0; // 실제 YAML 값으로 업데이트
    }

    /**
     * 원점 Y 좌표 동적 반환 (YAML에서 로딩)
     */
    public double getOriginY() {
        MapMetadata metadata = getMetadata();
        return metadata != null ? metadata.getOriginY() : -22.1; // 실제 YAML 값으로 업데이트
    }

    // =====================================
    // BaseS3Properties 구현
    // =====================================

    @Override
    public String generateS3Key(String fileType) {
        String filename = fileNames.get(fileType);
        if (filename == null) {
            throw new IllegalArgumentException("지원하지 않는 파일 타입: " + fileType);
        }
        
        // mapId 치환 없이 filename만 치환
        return s3PathPattern.replace("{filename}", filename);
    }

    @Override
    public String getFileName(String fileType) {
        String filename = fileNames.get(fileType);
        if (filename == null) {
            throw new IllegalArgumentException("지원하지 않는 파일 타입: " + fileType);
        }
        return filename;
    }

    @Override
    public boolean isSupported(String fileType) {
        return fileNames.containsKey(fileType);
    }
}
