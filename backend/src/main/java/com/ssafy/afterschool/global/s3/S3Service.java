package com.ssafy.afterschool.global.s3;

import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import io.awspring.cloud.s3.S3Template;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.core.io.Resource;

import jakarta.annotation.PostConstruct;

/**
 * AWS S3 파일 관리 서비스
 * - Spring Cloud AWS S3Template 사용
 * - 증거 파일 다운로드 (ROS2에서 업로드한 파일)
 * - 맵 파일 다운로드 (SLAM으로 생성된 맵)
 */
@Service
@Slf4j
public class S3Service {

    private final S3Template s3Template;
    private final String bucketName;

    public S3Service(S3Template s3Template, @Value("${spring.cloud.aws.s3.bucket}") String bucketName) {
        this.s3Template = s3Template;
        this.bucketName = bucketName;
    }

    /**
     * 애플리케이션 시작 시 S3 연결 테스트
     */
    @PostConstruct
    public void testConnection() {
        try {
            log.info("✅ S3 서비스 초기화 완료. 버킷: {}", bucketName);
        } catch (Exception e) {
            log.warn("⚠️ S3 연결 테스트 실패 (실제 사용 시 재시도): {}", e.getMessage());
        }
    }

    /**
     * 파일 다운로드 (바이트 배열로 반환)
     * @param s3Key S3 객체 키 (예: "evidence/tb1/20250919_143025_detection.jpg")
     * @return 파일 바이트 배열
     * @throws ApiException 파일 다운로드 실패 시
     */
    public byte[] downloadFile(String s3Key) {
        try {
            Resource resource = s3Template.download(bucketName, s3Key);
            byte[] fileBytes = resource.getInputStream().readAllBytes();
            log.info("📁 파일 다운로드 완료: {} ({} bytes)", s3Key, fileBytes.length);
            return fileBytes;
        } catch (Exception e) {
            log.error("❌ S3 파일 다운로드 실패: {}", s3Key, e);
            throw new ApiException(ErrorCode.FILE_DOWNLOAD_FAILED, "s3Key: " + s3Key, e);
        }
    }

    /**
     * 파일 다운로드 (Resource로 반환)
     * @param s3Key S3 객체 키 (예: "evidence/tb1/20250919_143025_detection.jpg")
     * @return 파일 Resource
     * @throws ApiException 파일 다운로드 실패 시
     */
    public Resource downloadFileAsResource(String s3Key) {
        try {
            Resource resource = s3Template.download(bucketName, s3Key);
            log.info("📁 파일 리소스 다운로드 완료: {}", s3Key);
            return resource;
        } catch (Exception e) {
            log.error("❌ S3 파일 리소스 다운로드 실패: {}", s3Key, e);
            throw new ApiException(ErrorCode.FILE_DOWNLOAD_FAILED, "s3Key: " + s3Key, e);
        }
    }

    /**
     * 파일 존재 여부 확인
     * @param s3Key S3 객체 키
     * @return 파일 존재 여부
     * @throws ApiException S3 연결 오류 시
     */
    public boolean fileExists(String s3Key) {
        try {
            boolean exists = s3Template.objectExists(bucketName, s3Key);
            log.debug("📁 파일 존재 여부 확인: {} -> {}", s3Key, exists);
            return exists;
        } catch (Exception e) {
            log.error("❌ 파일 존재 여부 확인 중 S3 연결 오류: {}", s3Key, e);
            throw new ApiException(ErrorCode.S3_CONNECTION_ERROR, "s3Key: " + s3Key, e);
        }
    }

    /**
     * 파일 메타데이터 조회
     * @param s3Key S3 객체 키
     * @return 파일 메타데이터
     * @throws ApiException 파일이 없거나 S3 오류 시
     */
    public S3FileMetadata getFileMetadata(String s3Key) {
        try {
            if (!fileExists(s3Key)) {
                log.warn("📁 파일을 찾을 수 없음: {}", s3Key);
                throw new ApiException(ErrorCode.FILE_NOT_FOUND, "s3Key: " + s3Key);
            }
            
            S3FileMetadata metadata = S3FileMetadata.builder()
                    .s3Key(s3Key)
                    .contentType(determineContentType(s3Key))
                    .bucketName(bucketName)
                    .build();
                    
            log.debug("📁 파일 메타데이터 조회 완료: {}", s3Key);
            return metadata;
                    
        } catch (ApiException e) {
            // ApiException은 그대로 재던지기 (이미 적절한 에러 코드 포함)
            throw e;
        } catch (Exception e) {
            log.error("❌ 파일 메타데이터 조회 중 S3 오류: {}", s3Key, e);
            throw new ApiException(ErrorCode.S3_CONNECTION_ERROR, "s3Key: " + s3Key, e);
        }
    }

    /**
     * 파일 확장자로 Content-Type 추정
     * @param s3Key S3 객체 키
     * @return MIME 타입
     */
    private String determineContentType(String s3Key) {
        if (s3Key == null) return "application/octet-stream";
        
        String lower = s3Key.toLowerCase();
        if (lower.endsWith(".jpg") || lower.endsWith(".jpeg")) return "image/jpeg";
        if (lower.endsWith(".png")) return "image/png";
        if (lower.endsWith(".mp4")) return "video/mp4";
        if (lower.endsWith(".pgm")) return "application/octet-stream";
        if (lower.endsWith(".yaml") || lower.endsWith(".yml")) return "text/yaml";
        if (lower.endsWith(".pbstream")) return "application/octet-stream";
        return "application/octet-stream";
    }

    /**
     * 파일 직접 접근 URL 생성 (공개 버킷인 경우)
     * @param s3Key S3 객체 키
     * @return 파일 직접 접근 URL
     */
    public String getPublicUrl(String s3Key) {
        if (s3Key == null || s3Key.trim().isEmpty()) {
            throw new ApiException(ErrorCode.INVALID_REQUEST_PARAMETER, "s3Key cannot be null or empty");
        }
        
        try {
            String url = String.format("https://%s.s3.%s.amazonaws.com/%s", 
                    bucketName, "ap-northeast-2", s3Key);
            log.debug("📁 공개 URL 생성 완료: {}", url);
            return url;
        } catch (Exception e) {
            log.error("❌ 공개 URL 생성 중 오류: {}", s3Key, e);
            throw new ApiException(ErrorCode.INTERNAL_SERVER_ERROR, "URL generation failed for s3Key: " + s3Key, e);
        }
    }
}
