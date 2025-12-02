package com.ssafy.afterschool.domain.event.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

/**
 * 증거 파일 응답 DTO
 */
@Getter
@Builder
public class EvidenceFileResponse {

    private Long id;

    @JsonProperty("eventId")
    private Long eventId;

    @JsonProperty("fileType")
    private String fileType; // image, video 등 (mimeType에서 추출)

    @JsonProperty("s3Bucket")
    private String s3Bucket;

    @JsonProperty("s3Key")
    private String s3Key;

    @JsonProperty("originalFilename")
    private String originalFilename;

    @JsonProperty("fileSize")
    private Long fileSize;

    @JsonProperty("mimeType")
    private String mimeType;

    @JsonProperty("metadata")
    private String metadata;

    @JsonProperty("createdAt")
    private LocalDateTime createdAt;
}
