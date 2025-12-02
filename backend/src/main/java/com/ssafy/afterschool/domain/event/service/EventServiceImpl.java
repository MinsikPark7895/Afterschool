package com.ssafy.afterschool.domain.event.service;

import com.ssafy.afterschool.domain.event.dto.*;
import com.ssafy.afterschool.domain.event.entity.Event;
import com.ssafy.afterschool.domain.event.entity.EvidenceFile;
import com.ssafy.afterschool.domain.event.repository.EventRepository;
import com.ssafy.afterschool.domain.event.repository.EvidenceFileRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.response.PageResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

/**
 * 이벤트 서비스 구현체
 */
@Slf4j
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class EventServiceImpl implements EventService {

    private final EventRepository eventRepository;
    private final EvidenceFileRepository evidenceFileRepository;

    @Override
    public PageResponse<EventResponse> getDetectionEvents(EventListRequest request) {
        // 페이징 설정 (항상 최신순 정렬)
        Pageable pageable = PageRequest.of(request.getPage(), request.getSize(),
            Sort.by(Sort.Direction.DESC, "createdAt"));

        // 날짜 범위 변환
        LocalDateTime startDateTime = request.getStartDate() != null ?
            request.getStartDate().atStartOfDay() : null;
        LocalDateTime endDateTime = request.getEndDate() != null ?
            request.getEndDate().atTime(23, 59, 59) : null;

        // 침입자 탐지 이벤트만 필터링 조회
        Page<Event> eventPage = eventRepository.findDetectionEventsByFilters(
            request.getRobotId(),
            request.getSeverity() != null ? request.getSeverity().name() : null,
            startDateTime,
            endDateTime,
            pageable
        );

        // DTO 변환
        List<EventResponse> eventResponses = eventPage.getContent().stream()
            .map(this::convertToEventResponse)
            .collect(Collectors.toList());

        return PageResponse.of(eventPage, eventResponses);
    }

    @Override
    public EventResponse getEvent(Long eventId) {
        Event event = eventRepository.findById(eventId)
            .orElseThrow(() -> new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "이벤트를 찾을 수 없습니다."));

        return convertToEventResponseWithFiles(event);
    }


    @Override
    public List<EvidenceFileResponse> getEvidenceFiles(Long eventId) {
        // 이벤트 존재 확인
        if (!eventRepository.existsById(eventId)) {
            throw new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "이벤트를 찾을 수 없습니다.");
        }

        List<EvidenceFile> files = evidenceFileRepository.findByEventIdOrderByCreatedAtAsc(eventId);
        return files.stream()
            .map(this::convertToEvidenceFileResponse)
            .collect(Collectors.toList());
    }

    @Override
    public EvidenceFileResponse getEvidenceFile(Long fileId) {
        EvidenceFile file = evidenceFileRepository.findById(fileId)
            .orElseThrow(() -> new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "증거 파일을 찾을 수 없습니다."));

        return convertToEvidenceFileResponse(file);
    }


    // === Private Methods ===

    private Sort parseSort(String sortString) {
        if (sortString == null || sortString.trim().isEmpty()) {
            return Sort.by(Sort.Direction.DESC, "createdAt");
        }

        String[] parts = sortString.split(",");
        String property = parts[0];
        Sort.Direction direction = parts.length > 1 && "asc".equalsIgnoreCase(parts[1]) ? 
            Sort.Direction.ASC : Sort.Direction.DESC;

        return Sort.by(direction, property);
    }

    private EventResponse convertToEventResponse(Event event) {
        return EventResponse.builder()
            .id(event.getId())
            .eventType(event.getEventType())
            .robotId(event.getRobotId())
            .severity(event.getSeverity())
            .locationData(event.getLocationData())
            .detectionData(event.getDetectionData())
            .createdAt(event.getCreatedAt())
            .build();
    }

    private EventResponse convertToEventResponseWithFiles(Event event) {
        List<EvidenceFile> files = evidenceFileRepository.findByEventIdOrderByCreatedAtAsc(event.getId());
        List<EvidenceFileResponse> fileResponses = files.stream()
            .map(this::convertToEvidenceFileResponse)
            .collect(Collectors.toList());

        return EventResponse.builder()
            .id(event.getId())
            .eventType(event.getEventType())
            .robotId(event.getRobotId())
            .severity(event.getSeverity())
            .locationData(event.getLocationData())
            .detectionData(event.getDetectionData())
            .createdAt(event.getCreatedAt())
            .evidenceFiles(fileResponses)
            .build();
    }

    private EvidenceFileResponse convertToEvidenceFileResponse(EvidenceFile file) {
        return EvidenceFileResponse.builder()
            .id(file.getId())
            .eventId(file.getEventId())
            .fileType(extractFileType(file.getMimeType()))
            .s3Bucket(file.getS3Bucket())
            .s3Key(file.getS3Key())
            .originalFilename(file.getOriginalFilename())
            .fileSize(file.getFileSize())
            .mimeType(file.getMimeType())
            .metadata(file.getMetadata())
            .createdAt(file.getCreatedAt())
            .build();
    }

    private String extractFileType(String mimeType) {
        if (mimeType.startsWith("image/")) return "image";
        if (mimeType.startsWith("video/")) return "video";
        return "file";
    }
}
