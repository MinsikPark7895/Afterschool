package com.ssafy.afterschool.domain.event.service;

import com.ssafy.afterschool.domain.event.dto.*;
import com.ssafy.afterschool.domain.event.entity.Event;
import com.ssafy.afterschool.global.response.PageResponse;
import org.springframework.data.domain.Pageable;

import java.util.List;

/**
 * 침입자 탐지 이벤트 서비스 인터페이스
 */
public interface EventService {

    /**
     * 침입자 탐지 이벤트 목록 조회 (페이징)
     */
    PageResponse<EventResponse> getDetectionEvents(EventListRequest request);

    /**
     * 이벤트 상세 조회 (증거 파일 포함)
     */
    EventResponse getEvent(Long eventId);

    /**
     * 증거 파일 목록 조회
     */
    List<EvidenceFileResponse> getEvidenceFiles(Long eventId);

    /**
     * 증거 파일 상세 조회 (다운로드용)
     */
    EvidenceFileResponse getEvidenceFile(Long fileId);

}
