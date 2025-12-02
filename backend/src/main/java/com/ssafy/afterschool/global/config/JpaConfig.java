package com.ssafy.afterschool.global.config;

import org.springframework.context.annotation.Configuration;
import org.springframework.data.jpa.repository.config.EnableJpaAuditing;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;

/**
 * JPA 관련 설정
 * - Entity 스캔 경로 설정
 * - Repository 스캔 경로 설정
 * - JPA Auditing 활성화 (생성일, 수정일 자동 관리)
 */
@Configuration
@EnableJpaAuditing
@EnableJpaRepositories(basePackages = "com.ssafy.afterschool.domain.*.repository")
public class JpaConfig {
}
