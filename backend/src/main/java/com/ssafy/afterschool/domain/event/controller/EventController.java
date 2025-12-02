package com.ssafy.afterschool.domain.event.controller;

import com.ssafy.afterschool.domain.event.dto.EventListRequest;
import com.ssafy.afterschool.domain.event.dto.EventResponse;
import com.ssafy.afterschool.domain.event.dto.EvidenceFileResponse;
import com.ssafy.afterschool.domain.event.service.EventService;
import com.ssafy.afterschool.global.constants.SuccessCode;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.response.PageResponse;
import com.ssafy.afterschool.global.s3.S3Service;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.core.io.Resource;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 침입자 탐지 이벤트 컨트롤러
 * - 침입자 탐지 이벤트 목록 및 상세 조회
 * - 증거 파일 다운로드
 */
@Tag(name = "Event", description = "이벤트 관리 API")
@Slf4j
@RestController
@RequestMapping("/events")
@RequiredArgsConstructor
public class EventController {

    private final EventService eventService;
    private final S3Service s3Service;

    @Operation(summary = "침입자 탐지 이벤트 목록 조회", description = "침입자 탐지 이벤트 목록을 조회합니다.")
    @GetMapping
    public ApiResponse<PageResponse<EventResponse>> getDetectionEvents(
            @ModelAttribute EventListRequest request) {

        log.info("📋 침입자 탐지 이벤트 목록 조회 요청 - Robot: {}, Page: {}",
            request.getRobotId(), request.getPage());

        PageResponse<EventResponse> events = eventService.getDetectionEvents(request);
        return ApiResponse.success(SuccessCode.EVENT_LIST_SUCCESS, events);
    }

    @Operation(summary = "이벤트 상세 조회", description = "특정 이벤트의 상세 정보 및 증거 파일 정보를 조회합니다.")
    @GetMapping("/{eventId}")
    public ApiResponse<EventResponse> getEvent(
            @Parameter(description = "이벤트 ID") @PathVariable Long eventId) {
        
        log.info("🔍 이벤트 상세 조회 - ID: {}", eventId);

        EventResponse event = eventService.getEvent(eventId);
        return ApiResponse.success(SuccessCode.EVENT_DETAIL_SUCCESS, event);
    }

    @Operation(summary = "이벤트 증거 파일 목록", description = "특정 이벤트의 증거 파일 목록을 조회합니다.")
    @GetMapping("/evidence/{eventId}")
    public ApiResponse<List<EvidenceFileResponse>> getEvidenceFiles(
            @Parameter(description = "이벤트 ID") @PathVariable Long eventId) {
        
        log.info("📁 증거 파일 목록 조회 - EventID: {}", eventId);

        List<EvidenceFileResponse> files = eventService.getEvidenceFiles(eventId);
        return ApiResponse.success(SuccessCode.FILE_DOWNLOAD_SUCCESS, files);
    }

    @Operation(summary = "증거 파일 다운로드", description = "S3에 저장된 증거 파일을 다운로드합니다.")
    @GetMapping("/evidence/download/{fileId}")
    public ResponseEntity<Resource> downloadEvidenceFile(
            @Parameter(description = "증거 파일 ID") @PathVariable Long fileId) {

        log.info("⬇️ 증거 파일 다운로드 - FileID: {}", fileId);

        // 파일 정보 조회
        EvidenceFileResponse fileInfo = eventService.getEvidenceFile(fileId);

        // S3에서 파일 다운로드
        Resource resource = s3Service.downloadFileAsResource(fileInfo.getS3Key());

        return ResponseEntity.ok()
            .contentType(MediaType.parseMediaType(fileInfo.getMimeType()))
            .header(HttpHeaders.CONTENT_DISPOSITION,
                "attachment; filename=\"" + fileInfo.getOriginalFilename() + "\"")
            .header("X-File-Size", String.valueOf(fileInfo.getFileSize()))
            .header("X-S3-Key", fileInfo.getS3Key())
            .body(resource);
    }
}
