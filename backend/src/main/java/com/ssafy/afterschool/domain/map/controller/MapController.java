package com.ssafy.afterschool.domain.map.controller;

import com.ssafy.afterschool.domain.map.service.MapService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.responses.ApiResponse;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.core.io.ByteArrayResource;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

/**
 * 맵 파일 다운로드 API
 * - ROS2 SLAM으로 생성된 맵 파일들을 S3에서 다운로드
 * - 프론트엔드에서 맵 렌더링용으로 사용
 * - pbstream은 업로드되지 않으므로 제외
 */
@Tag(name = "맵 관리", description = "맵 파일 다운로드 API")
@RestController
@RequestMapping("/maps")
@RequiredArgsConstructor
@Slf4j
public class MapController {

    private final MapService mapService;

    @Operation(
            summary = "맵 이미지 파일 다운로드",
            description = "SLAM으로 생성된 .pgm 맵 이미지 파일을 S3에서 다운로드합니다. 프론트엔드에서 Canvas 렌더링용으로 사용됩니다."
    )
    @ApiResponses({
            @ApiResponse(responseCode = "200", description = "맵 이미지 파일 다운로드 성공"),
            @ApiResponse(responseCode = "404", description = "맵 파일을 찾을 수 없음"),
            @ApiResponse(responseCode = "503", description = "S3 연결 오류")
    })
    @GetMapping("/pgm")
    public ResponseEntity<ByteArrayResource> downloadMapImage() {
        log.info("🗺️ 맵 이미지 파일 다운로드 요청");
        
        byte[] fileData = mapService.downloadMapFile("pgm");
        String filename = mapService.getMapFileName("pgm");
        
        return ResponseEntity.ok()
                .header(HttpHeaders.CONTENT_DISPOSITION, "attachment; filename=\"" + filename + "\"")
                .contentType(MediaType.APPLICATION_OCTET_STREAM)
                .contentLength(fileData.length)
                .body(new ByteArrayResource(fileData));
    }

    @Operation(
            summary = "맵 설정 파일 다운로드",
            description = "맵 메타데이터가 포함된 .yaml 설정 파일을 S3에서 다운로드합니다. 해상도, 원점 좌표 등의 정보가 포함되어 있습니다."
    )
    @ApiResponses({
            @ApiResponse(responseCode = "200", description = "맵 설정 파일 다운로드 성공"),
            @ApiResponse(responseCode = "404", description = "맵 파일을 찾을 수 없음"),
            @ApiResponse(responseCode = "503", description = "S3 연결 오류")
    })
    @GetMapping("/yaml")
    public ResponseEntity<ByteArrayResource> downloadMapConfig() {
        log.info("🗺️ 맵 설정 파일 다운로드 요청");
        
        byte[] fileData = mapService.downloadMapFile("yaml");
        String filename = mapService.getMapFileName("yaml");
        
        return ResponseEntity.ok()
                .header(HttpHeaders.CONTENT_DISPOSITION, "attachment; filename=\"" + filename + "\"")
                .contentType(MediaType.parseMediaType("text/yaml"))
                .contentLength(fileData.length)
                .body(new ByteArrayResource(fileData));
    }
}
