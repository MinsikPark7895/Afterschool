package com.ssafy.afterschool.global.response;

import lombok.Builder;
import lombok.Getter;
import org.springframework.data.domain.Page;

import java.util.List;

/**
 * 페이징 응답 DTO - 깔끔한 페이징 정보만 포함
 */
@Getter
@Builder
public class PageResponse<T> {

    /**
     * 실제 데이터 목록
     */
    private final List<T> content;

    /**
     * 현재 페이지 번호 (0부터 시작)
     */
    private final int page;

    /**
     * 페이지 크기
     */
    private final int size;

    /**
     * 전체 요소 수
     */
    private final long totalElements;

    /**
     * 전체 페이지 수
     */
    private final int totalPages;

    /**
     * 첫 번째 페이지 여부
     */
    private final boolean first;

    /**
     * 마지막 페이지 여부
     */
    private final boolean last;

    /**
     * Spring Data Page 객체를 PageResponse로 변환
     */
    public static <T> PageResponse<T> of(Page<T> page) {
        return PageResponse.<T>builder()
                .content(page.getContent())
                .page(page.getNumber())
                .size(page.getSize())
                .totalElements(page.getTotalElements())
                .totalPages(page.getTotalPages())
                .first(page.isFirst())
                .last(page.isLast())
                .build();
    }

    /**
     * Spring Data Page 객체와 변환된 컨텐츠로 PageResponse 생성
     */
    public static <T, U> PageResponse<U> of(Page<T> page, List<U> convertedContent) {
        return PageResponse.<U>builder()
                .content(convertedContent)
                .page(page.getNumber())
                .size(page.getSize())
                .totalElements(page.getTotalElements())
                .totalPages(page.getTotalPages())
                .first(page.isFirst())
                .last(page.isLast())
                .build();
    }
}