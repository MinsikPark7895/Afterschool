package com.ssafy.afterschool.domain.event.dto;

import com.ssafy.afterschool.domain.event.entity.Severity;
import lombok.Getter;
import lombok.Setter;
import org.springframework.format.annotation.DateTimeFormat;

import java.time.LocalDate;

/**
 * 침입자 탐지 이벤트 목록 조회 요청 DTO
 */
@Getter
@Setter
public class EventListRequest {

    private Severity severity;

    private String robotId; // tb1, tb2

    @DateTimeFormat(pattern = "yyyy-MM-dd")
    private LocalDate startDate;

    @DateTimeFormat(pattern = "yyyy-MM-dd")
    private LocalDate endDate;

    private int page = 0; // 페이지 번호 (0부터 시작)

    private int size = 20; // 페이지 크기
}
