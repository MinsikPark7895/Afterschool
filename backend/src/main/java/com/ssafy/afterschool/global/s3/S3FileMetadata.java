package com.ssafy.afterschool.global.s3;

import lombok.Builder;
import lombok.Getter;

import java.time.Instant;

/**
 * S3 파일 메타데이터 DTO
 */
@Getter
@Builder
public class S3FileMetadata {
    private final String s3Key;
    private final String contentType;
    private final String bucketName;
    private final Long contentLength;
    private final Instant lastModified;
}
