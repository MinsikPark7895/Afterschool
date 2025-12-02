package com.ssafy.afterschool.domain.map.config;

import com.ssafy.afterschool.domain.map.service.MapMetadataLoader;
import com.ssafy.afterschool.global.properties.domain.MapProperties;
import lombok.extern.slf4j.Slf4j;
import org.springframework.context.annotation.Configuration;

import jakarta.annotation.PostConstruct;

/**
 * 맵 도메인 설정 클래스
 * MapProperties에 MapMetadataLoader 의존성을 주입
 */
@Configuration
@Slf4j
public class MapConfig {

    private final MapProperties mapProperties;
    private final MapMetadataLoader mapMetadataLoader;

    public MapConfig(MapProperties mapProperties, MapMetadataLoader mapMetadataLoader) {
        this.mapProperties = mapProperties;
        this.mapMetadataLoader = mapMetadataLoader;
    }

    @PostConstruct
    public void init() {
        log.info("🗺️ MapConfig 초기화: MapProperties에 MetadataLoader 주입");
        mapProperties.setMetadataLoader(mapMetadataLoader);
    }
}
