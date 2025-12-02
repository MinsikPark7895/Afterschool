package com.ssafy.afterschool.global.properties.template;

/**
 * MQTT 토픽 관리를 위한 기본 인터페이스
 * 로봇 도메인에서 MQTT 토픽 생성이 필요한 경우 구현
 */
public interface BaseMqttProperties {

    /**
     * MQTT 토픽 생성
     * @param params 토픽 생성에 필요한 파라미터들
     * @return MQTT 토픽 문자열
     */
    String generateTopic(String... params);

    /**
     * 토픽 패턴 유효성 검증
     * @param topic 검증할 토픽
     * @return 유효 여부
     */
    boolean isValidTopic(String topic);
}
