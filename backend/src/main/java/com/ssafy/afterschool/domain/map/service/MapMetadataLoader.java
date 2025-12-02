package com.ssafy.afterschool.domain.map.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.ssafy.afterschool.domain.map.dto.MapMetadata;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.s3.S3Service;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.cache.annotation.Cacheable;
import org.springframework.stereotype.Service;

import java.io.IOException;

/**
 * S3에서 맵 YAML 파일을 다운로드하여 메타데이터를 추출하는 서비스
 */
@Slf4j
@Service
@RequiredArgsConstructor
public class MapMetadataLoader {
    
    private final S3Service s3Service;
    private final ObjectMapper yamlObjectMapper = new ObjectMapper(new YAMLFactory());
    
    /**
     * S3에서 맵 YAML 파일을 다운로드하고 메타데이터를 파싱
     * 
     * @param mapId 맵 ID (실제로는 사용하지 않음, 단일 맵이므로)
     * @return 파싱된 맵 메타데이터
     * @throws ApiException YAML 파일이 없거나 파싱 실패 시
     */
    @Cacheable(value = "mapMetadata", key = "#mapId")
    public MapMetadata loadMapMetadata(String mapId) throws IOException {
        log.info("🗺️ 맵 메타데이터 로딩 시작: mapId={}", mapId);
        
        try {
            // S3에서 YAML 파일 다운로드 (고정 경로)
            String yamlKey = "maps/map.yaml";
            log.debug("📥 S3에서 YAML 파일 다운로드 시도: key={}", yamlKey);
            
            byte[] yamlContent = s3Service.downloadFile(yamlKey);
            
            // YAML 파싱
            log.debug("📄 YAML 파일 파싱 시작");
            MapMetadata metadata = yamlObjectMapper.readValue(yamlContent, MapMetadata.class);
            
            // 유효성 검증
            if (!metadata.isValid()) {
                log.error("❌ 유효하지 않은 맵 메타데이터: mapId={}, metadata={}", mapId, metadata);
                throw new ApiException(ErrorCode.MAP_FILE_CORRUPTED);
            }
            
            log.info("✅ 맵 메타데이터 로딩 성공: mapId={}, resolution={}, originX={}, originY={}", 
                mapId, metadata.resolution(), metadata.getOriginX(), metadata.getOriginY());
            
            return metadata;
            
        } catch (Exception e) {
            if (e instanceof ApiException) {
                throw e;
            }
            
            log.error("❌ 맵 메타데이터 로딩 실패: mapId={}", mapId, e);
            throw new ApiException(ErrorCode.MAP_FILE_CORRUPTED);
        }
    }
}
