package com.ssafy.afterschool.domain.event.service;

import com.ssafy.afterschool.domain.event.entity.Event;
import com.ssafy.afterschool.domain.event.entity.EventType;
import com.ssafy.afterschool.domain.event.entity.Severity;
import com.ssafy.afterschool.domain.event.entity.EvidenceFile;
import com.ssafy.afterschool.domain.event.repository.EventRepository;
import com.ssafy.afterschool.domain.event.repository.EvidenceFileRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

/**
 * 이벤트 로깅 전용 서비스 구현체
 * - AOP에서 MQTT 이벤트를 자동으로 기록할 때 사용
 * - 단순 CRUD 로직만 담당 (비즈니스 로직 X)
 */
@Slf4j
@Service
@RequiredArgsConstructor
@Transactional
public class EventLogServiceImpl implements EventLogService {

    private final EventRepository eventRepository;
    private final EvidenceFileRepository evidenceFileRepository;

    @Override
    public Event createEvent(String eventType, String robotId, String severity,
                           String locationData, String detectionData) {
        Event event = Event.builder()
            .eventType(EventType.valueOf(eventType.toUpperCase()))
            .robotId(robotId)
            .severity(Severity.valueOf(severity.toUpperCase()))
            .locationData(locationData)
            .detectionData(detectionData)
            .build();

        Event savedEvent = eventRepository.save(event);
        log.info("📝 이벤트 로그 기록 완료 - ID: {}, Type: {}, Robot: {}",
            savedEvent.getId(), eventType, robotId);

        return savedEvent;
    }

    @Override
    public void createEvidenceFile(Long eventId, String s3Bucket, String s3Key,
                                 String originalFilename, Long fileSize, String mimeType, String metadata) {
        // 이벤트 존재 확인
        if (!eventRepository.existsById(eventId)) {
            throw new ApiException(ErrorCode.RESOURCE_NOT_FOUND, "이벤트를 찾을 수 없습니다.");
        }

        EvidenceFile file = EvidenceFile.builder()
            .eventId(eventId)
            .s3Bucket(s3Bucket)
            .s3Key(s3Key)
            .originalFilename(originalFilename)
            .fileSize(fileSize)
            .mimeType(mimeType)
            .metadata(metadata)
            .build();

        evidenceFileRepository.save(file);
        log.info("📁 증거 파일 로그 기록 완료 - EventID: {}, Filename: {}", eventId, originalFilename);
    }
}