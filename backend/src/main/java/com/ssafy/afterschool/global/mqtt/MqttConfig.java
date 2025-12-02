package com.ssafy.afterschool.global.mqtt;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.integration.mqtt.support.DefaultPahoMessageConverter;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;

/**
 * MQTT 설정 클래스
 * - ROS2와 Spring Boot 간 MQTT 통신 브릿지
 * - Eclipse Paho MQTT 클라이언트 기반
 * - 안정적인 연결 및 재연결 로직 포함
 */
@Slf4j
@Configuration
@RequiredArgsConstructor
public class MqttConfig {

    @Value("${mqtt.broker.url}")
    private String brokerUrl;

    @Value("${mqtt.broker.username:}")
    private String username;

    @Value("${mqtt.broker.password:}")
    private String password;

    private static final String CLIENT_ID_PREFIX = "afterschool-backend";
    private static final int CONNECTION_TIMEOUT = 30;
    private static final int KEEP_ALIVE_INTERVAL = 60;
    private static final int MAX_INFLIGHT = 10;

    /**
     * MQTT 클라이언트 팩토리 설정
     * - 안정적인 연결을 위한 옵션 구성
     * - 자동 재연결 및 세션 관리
     */
    @Bean
    public MqttPahoClientFactory mqttClientFactory() {
        log.info("🔧 MQTT 클라이언트 팩토리 설정 시작");
        
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        MqttConnectOptions options = new MqttConnectOptions();
        
        // 기본 연결 설정
        options.setServerURIs(new String[]{brokerUrl});
        options.setConnectionTimeout(CONNECTION_TIMEOUT);
        options.setKeepAliveInterval(KEEP_ALIVE_INTERVAL);
        options.setMaxInflight(MAX_INFLIGHT);
        
        // 세션 및 재연결 설정
        options.setCleanSession(true);           // 새 세션으로 시작
        options.setAutomaticReconnect(true);     // 자동 재연결 활성화
        
        // 인증 설정 (필요시)
        if (!username.isEmpty()) {
            options.setUserName(username);
            if (!password.isEmpty()) {
                options.setPassword(password.toCharArray());
            }
        }
        
        factory.setConnectionOptions(options);
        
        log.info("✅ MQTT 클라이언트 팩토리 설정 완료 - Broker: {}", brokerUrl);
        return factory;
    }

    /**
     * MQTT 메시지 수신용 입력 채널
     * - ROS2에서 오는 메시지를 처리
     */
    @Bean
    public MessageChannel mqttInputChannel() {
        return new DirectChannel();
    }

    /**
     * MQTT 메시지 발송용 출력 채널  
     * - ROS2로 명령을 전송
     */
    @Bean
    public MessageChannel mqttOutputChannel() {
        return new DirectChannel();
    }

    /**
     * MQTT 메시지 수신 어댑터
     * - from_robot/* 토픽을 구독하여 메시지 수신
     * - ROS2에서 전송하는 로봇 상태 및 이벤트 처리
     */
    @Bean
    public MqttPahoMessageDrivenChannelAdapter mqttInbound() {
        log.info("🔧 MQTT 인바운드 어댑터 설정 시작");
        
        String clientId = CLIENT_ID_PREFIX + "-subscriber-" + System.currentTimeMillis();
        
        // 구독할 토픽 패턴 정의
        String[] topics = {
            MqttTopics.FROM_ROBOT_STATUS_BASIC + "+",        // 로봇 기본 상태 (tb1, tb2)
            MqttTopics.FROM_ROBOT_STATUS_DETAIL + "+",       // 로봇 상세 상태
            MqttTopics.FROM_ROBOT_EVENT_DETECTION + "+",     // 침입자 탐지 이벤트
            MqttTopics.FROM_ROBOT_EVENT_MISSION + "+",  // 임무 완료 이벤트
            MqttTopics.FROM_ROBOT_SYSTEM_STATUS // 시스템 상태 이벤트
        };
        
        MqttPahoMessageDrivenChannelAdapter adapter = new MqttPahoMessageDrivenChannelAdapter(
            brokerUrl, 
            clientId, 
            mqttClientFactory(),
            topics
        );
        
        // 메시지 컨버터 설정
        adapter.setConverter(new DefaultPahoMessageConverter());
        adapter.setQos(1);  // QoS 1 (적어도 한 번 전달)
        adapter.setOutputChannel(mqttInputChannel());
        adapter.setCompletionTimeout(5000);
        
        log.info("✅ MQTT 인바운드 어댑터 설정 완료 - Topics: {}", String.join(", ", topics));
        return adapter;
    }

    /**
     * MQTT 메시지 발송 핸들러
     * - to_robot/* 토픽으로 메시지 발송
     * - ROS2로 명령 전송
     */
    @Bean
    @ServiceActivator(inputChannel = "mqttOutputChannel")
    public MessageHandler mqttOutbound() {
        log.info("🔧 MQTT 아웃바운드 핸들러 설정 시작");
        
        String clientId = CLIENT_ID_PREFIX + "-publisher-" + System.currentTimeMillis();
        
        MqttPahoMessageHandler messageHandler = new MqttPahoMessageHandler(
            clientId, 
            mqttClientFactory()
        );
        
        messageHandler.setAsync(true);  // 비동기 전송
        messageHandler.setDefaultQos(1); // QoS 1
        messageHandler.setDefaultRetained(false); // 메시지 보관하지 않음
        
        log.info("✅ MQTT 아웃바운드 핸들러 설정 완료");
        return messageHandler;
    }

    // MQTT 메시지 처리는 MqttMessageRouter에서 담당
}
