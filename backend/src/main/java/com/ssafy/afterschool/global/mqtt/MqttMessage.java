package com.ssafy.afterschool.global.mqtt;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Builder;
import lombok.Data;

import java.util.Map;

/**
 * 표준 MQTT 메시지 형식
 * - 모든 MQTT 메시지의 통일된 구조
 * - 헤더와 페이로드 표준화
 */
@Data
@Builder
public class MqttMessage {

    private Header header;
    private Payload payload;

    @Data
    @Builder
    public static class Header {
        private String timestamp;

        @JsonProperty("message_id")
        private String messageId;

        @JsonProperty("robot_id")
        private String robotId;
    }

    @Data
    @Builder
    public static class Payload {
        @JsonProperty("command_type")
        private String commandType;

        private Map<String, Object> data;
    }
}
