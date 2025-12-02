package com.ssafy.afterschool.global.config;

import lombok.extern.slf4j.Slf4j;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.scheduling.annotation.EnableAsync;
import org.springframework.scheduling.concurrent.ThreadPoolTaskExecutor;

import java.util.concurrent.Executor;

/**
 * 비동기 처리 설정
 * - MQTT 이벤트 핸들러의 비동기 처리를 위한 ThreadPool 설정
 */
@Slf4j
@Configuration
@EnableAsync
public class AsyncConfig {

    /**
     * MQTT 이벤트 처리용 ThreadPool
     * - 로봇 상태 업데이트, 침입자 탐지 등의 이벤트를 비동기로 처리
     */
    @Bean(name = "taskExecutor")
    public Executor taskExecutor() {
        log.info("🔧 비동기 ThreadPool 설정 시작");
        
        ThreadPoolTaskExecutor executor = new ThreadPoolTaskExecutor();
        
        // 기본 스레드 개수
        executor.setCorePoolSize(5);
        
        // 최대 스레드 개수
        executor.setMaxPoolSize(20);
        
        // 큐 용량
        executor.setQueueCapacity(100);
        
        // 스레드 이름 접두사
        executor.setThreadNamePrefix("AsyncMqtt-");
        
        // 스레드가 유휴 상태로 유지되는 시간 (초)
        executor.setKeepAliveSeconds(60);
        
        // 애플리케이션 종료 시 큐에 남아있는 작업 완료 대기
        executor.setWaitForTasksToCompleteOnShutdown(true);
        
        // 대기 시간 (초)
        executor.setAwaitTerminationSeconds(10);
        
        // 스레드 풀 초기화
        executor.initialize();
        
        log.info("✅ 비동기 ThreadPool 설정 완료");
        return executor;
    }
}
