package com.ssafy.afterschool.domain.event.repository;

import com.ssafy.afterschool.domain.event.entity.EvidenceFile;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

/**
 * 증거 파일 Repository
 */
@Repository
public interface EvidenceFileRepository extends JpaRepository<EvidenceFile, Long> {

    /**
     * 이벤트ID로 증거 파일 목록 조회
     */
    List<EvidenceFile> findByEventIdOrderByCreatedAtAsc(Long eventId);

    /**
     * S3 키로 증거 파일 조회
     */
    EvidenceFile findByS3Key(String s3Key);

    /**
     * 특정 MIME 타입의 증거 파일 조회
     */
    List<EvidenceFile> findByEventIdAndMimeTypeStartingWithOrderByCreatedAtAsc(Long eventId, String mimeTypePrefix);

}
