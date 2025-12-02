package com.ssafy.afterschool.global.s3;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

/**
 * AWS S3 설정 클래스
 * - Spring Cloud AWS 자동 설정 사용
 * - application.yml의 cloud.aws 설정 활용
 */
@Configuration
public class S3Config {

    @Value("${spring.cloud.aws.s3.bucket}")
    private String bucketName;

    @Bean
    public String bucketName() {
        return bucketName;
    }
}
