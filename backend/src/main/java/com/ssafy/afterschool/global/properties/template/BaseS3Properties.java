package com.ssafy.afterschool.global.properties.template;

/**
 * S3 경로 관리를 위한 기본 인터페이스
 * 각 도메인에서 S3 파일 경로 생성이 필요한 경우 구현
 */
public interface BaseS3Properties {

    /**
     * S3 키 생성
     * @param identifier 식별자 (파일타입, 로봇ID, 이벤트ID 등)
     * @return S3 객체 키
     */
    String generateS3Key(String identifier);

    /**
     * 파일명 반환
     * @param identifier 식별자
     * @return 파일명
     */
    String getFileName(String identifier);

    /**
     * 지원하는 식별자인지 검증
     * @param identifier 검증할 식별자
     * @return 지원 여부
     */
    boolean isSupported(String identifier);
}
