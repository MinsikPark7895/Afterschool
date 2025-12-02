package com.ssafy.afterschool.domain.user.controller;

import com.ssafy.afterschool.domain.user.dto.UserCreateRequest;
import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.dto.UserUpdateRequest;
import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.domain.user.service.UserService;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Role;
import com.ssafy.afterschool.global.constants.Status;
import com.ssafy.afterschool.global.exception.ApiException;
import com.ssafy.afterschool.global.response.ApiResponse;
import com.ssafy.afterschool.global.response.PageResponse;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageImpl;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;

import java.time.LocalDateTime;
import java.util.List;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("AdminController 단위 테스트")
class AdminControllerTest {

    @Mock
    private UserService userService;

    @InjectMocks
    private AdminController adminController;

    private static final Long TEST_USER_ID = 1L;
    private static final String TEST_USERNAME = "testuser";
    private static final String TEST_PASSWORD = "password123";
    private static final String TEST_NAME = "Test User";

    private UserInfoResponse userInfoResponse;
    private UserCreateRequest userCreateRequest;
    private UserUpdateRequest userUpdateRequest;

    @BeforeEach
    void setUp() {
        userInfoResponse = UserInfoResponse.from(User.builder()
                .id(TEST_USER_ID)
                .username(TEST_USERNAME)
                .name(TEST_NAME)
                .role(Role.OPERATOR)
                .createdAt(LocalDateTime.now())
                .lastLoginAt(LocalDateTime.now())
                .build());

        userCreateRequest = new UserCreateRequest(TEST_USERNAME, TEST_PASSWORD, TEST_NAME, Role.OPERATOR);

        userUpdateRequest = new UserUpdateRequest("updateduser", "updatedpassword", "Updated User", Role.ADMIN);
    }

    @Nested
    @DisplayName("사용자 목록 조회 테스트")
    class GetAllUsersTest {

        @Test
        @DisplayName("사용자 목록 조회가 성공한다")
        void shouldGetAllUsersSuccessfully() {
            // given
            List<UserInfoResponse> users = List.of(userInfoResponse);
            Page<UserInfoResponse> userPage = new PageImpl<>(users, PageRequest.of(0, 20), 1);
            Pageable pageable = PageRequest.of(0, 20);

            given(userService.getAllUsers(any(Pageable.class)))
                    .willReturn(userPage);

            // when
            ApiResponse<PageResponse<UserInfoResponse>> response = adminController.getAllUsers(pageable);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("사용자 목록 조회가 완료되었습니다");
                        assertThat(r.data()).isNotNull();
                        assertThat(r.data().getContent()).hasSize(1);
                        assertThat(r.data().getContent().getFirst().getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.data().getTotalElements()).isEqualTo(1);
                    });

            then(userService).should().getAllUsers(any(Pageable.class));
        }

        @Test
        @DisplayName("올바른 Pageable 파라미터로 서비스가 호출된다")
        void shouldCallServiceWithCorrectPageableParameters() {
            // given
            Pageable pageable = PageRequest.of(0, 20);
            Page<UserInfoResponse> userPage = new PageImpl<>(List.of(), pageable, 0);
            given(userService.getAllUsers(pageable)).willReturn(userPage);

            // when
            adminController.getAllUsers(pageable);

            // then
            then(userService).should().getAllUsers(pageable);
        }
    }

    @Nested
    @DisplayName("사용자 상세 조회 테스트")
    class GetUserByIdTest {

        @Test
        @DisplayName("사용자 상세 조회가 성공한다")
        void shouldGetUserByIdSuccessfully() {
            // given
            given(userService.getUserById(TEST_USER_ID))
                    .willReturn(userInfoResponse);

            // when
            ApiResponse<UserInfoResponse> response = adminController.getUserById(TEST_USER_ID);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("프로필 조회가 완료되었습니다");
                        assertThat(r.data()).isNotNull();
                        assertThat(r.data().getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.data().getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.data().getName()).isEqualTo(TEST_NAME);
                    });

            then(userService).should().getUserById(TEST_USER_ID);
        }

        @Test
        @DisplayName("존재하지 않는 사용자 조회 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFound() {
            // given
            given(userService.getUserById(TEST_USER_ID))
                    .willThrow(new ApiException(ErrorCode.USER_NOT_FOUND));

            // when & then
            assertThatThrownBy(() -> adminController.getUserById(TEST_USER_ID))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(userService).should().getUserById(TEST_USER_ID);
        }
    }

    @Nested
    @DisplayName("사용자 생성 테스트")
    class CreateUserTest {

        @Test
        @DisplayName("사용자 생성이 성공한다")
        void shouldCreateUserSuccessfully() {
            // given
            given(userService.createUser(any(UserCreateRequest.class)))
                    .willReturn(userInfoResponse);

            // when
            ApiResponse<UserInfoResponse> response = adminController.createUser(userCreateRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("사용자가 성공적으로 생성되었습니다");
                        assertThat(r.data()).isNotNull();
                        assertThat(r.data().getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.data().getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.data().getName()).isEqualTo(TEST_NAME);
                    });

            then(userService).should().createUser(any(UserCreateRequest.class));
        }

        @Test
        @DisplayName("중복된 사용자명으로 생성 시 예외가 발생한다")
        void shouldThrowExceptionWhenUsernameAlreadyExists() {
            // given
            given(userService.createUser(any(UserCreateRequest.class)))
                    .willThrow(new ApiException(ErrorCode.USERNAME_ALREADY_EXISTS));

            // when & then
            assertThatThrownBy(() -> adminController.createUser(userCreateRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USERNAME_ALREADY_EXISTS);
                    });

            then(userService).should().createUser(any(UserCreateRequest.class));
        }

        @Test
        @DisplayName("올바른 요청 데이터로 서비스가 호출된다")
        void shouldCallServiceWithCorrectCreateRequest() {
            // given
            given(userService.createUser(userCreateRequest)).willReturn(userInfoResponse);

            // when
            adminController.createUser(userCreateRequest);

            // then
            then(userService).should().createUser(userCreateRequest);
        }
    }

    @Nested
    @DisplayName("사용자 수정 테스트")
    class UpdateUserTest {

        @Test
        @DisplayName("사용자 수정이 성공한다")
        void shouldUpdateUserSuccessfully() {
            // given
            UserInfoResponse updatedResponse = UserInfoResponse.from(User.builder()
                    .id(TEST_USER_ID)
                    .username("updateduser")
                    .name("Updated User")
                    .role(Role.ADMIN)
                    .createdAt(LocalDateTime.now())
                    .lastLoginAt(LocalDateTime.now())
                    .build());

            given(userService.updateUser(eq(TEST_USER_ID), any(UserUpdateRequest.class)))
                    .willReturn(updatedResponse);

            // when
            ApiResponse<UserInfoResponse> response = adminController.updateUser(TEST_USER_ID, userUpdateRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("사용자 정보가 성공적으로 수정되었습니다");
                        assertThat(r.data()).isNotNull();
                        assertThat(r.data().getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.data().getUsername()).isEqualTo("updateduser");
                        assertThat(r.data().getName()).isEqualTo("Updated User");
                        assertThat(r.data().getRole()).isEqualTo(Role.ADMIN);
                    });

            then(userService).should().updateUser(eq(TEST_USER_ID), any(UserUpdateRequest.class));
        }

        @Test
        @DisplayName("존재하지 않는 사용자 수정 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFoundForUpdate() {
            // given
            given(userService.updateUser(eq(TEST_USER_ID), any(UserUpdateRequest.class)))
                    .willThrow(new ApiException(ErrorCode.USER_NOT_FOUND));

            // when & then
            assertThatThrownBy(() -> adminController.updateUser(TEST_USER_ID, userUpdateRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(userService).should().updateUser(eq(TEST_USER_ID), any(UserUpdateRequest.class));
        }

        @Test
        @DisplayName("올바른 파라미터로 서비스가 호출된다")
        void shouldCallServiceWithCorrectUpdateParameters() {
            // given
            given(userService.updateUser(TEST_USER_ID, userUpdateRequest)).willReturn(userInfoResponse);

            // when
            adminController.updateUser(TEST_USER_ID, userUpdateRequest);

            // then
            then(userService).should().updateUser(TEST_USER_ID, userUpdateRequest);
        }
    }

    @Nested
    @DisplayName("사용자 삭제 테스트")
    class DeleteUserTest {

        @Test
        @DisplayName("사용자 삭제가 성공한다")
        void shouldDeleteUserSuccessfully() {
            // given
            willDoNothing().given(userService).deleteUser(TEST_USER_ID);

            // when
            ApiResponse<Void> response = adminController.deleteUser(TEST_USER_ID);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.status()).isEqualTo(Status.SUCCESS);
                        assertThat(r.message()).isEqualTo("사용자가 성공적으로 삭제되었습니다");
                        assertThat(r.data()).isNull();
                    });

            then(userService).should().deleteUser(TEST_USER_ID);
        }

        @Test
        @DisplayName("존재하지 않는 사용자 삭제 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFoundForDelete() {
            // given
            willThrow(new ApiException(ErrorCode.USER_NOT_FOUND))
                    .given(userService).deleteUser(TEST_USER_ID);

            // when & then
            assertThatThrownBy(() -> adminController.deleteUser(TEST_USER_ID))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(userService).should().deleteUser(TEST_USER_ID);
        }

        @Test
        @DisplayName("올바른 사용자 ID로 서비스가 호출된다")
        void shouldCallServiceWithCorrectUserId() {
            // given
            willDoNothing().given(userService).deleteUser(TEST_USER_ID);

            // when
            adminController.deleteUser(TEST_USER_ID);

            // then
            then(userService).should().deleteUser(TEST_USER_ID);
        }
    }

    @Nested
    @DisplayName("비즈니스 로직 호출 검증 테스트")
    class BusinessLogicInvocationTest {

        @Test
        @DisplayName("모든 메소드가 올바른 파라미터로 서비스를 호출한다")
        void shouldCallAllServiceMethodsWithCorrectParameters() {
            // given
            Pageable pageable = PageRequest.of(0, 20);
            Page<UserInfoResponse> userPage = new PageImpl<>(List.of(), pageable, 0);

            given(userService.getAllUsers(pageable)).willReturn(userPage);
            given(userService.getUserById(TEST_USER_ID)).willReturn(userInfoResponse);
            given(userService.createUser(userCreateRequest)).willReturn(userInfoResponse);
            given(userService.updateUser(TEST_USER_ID, userUpdateRequest)).willReturn(userInfoResponse);
            willDoNothing().given(userService).deleteUser(TEST_USER_ID);

            // when
            adminController.getAllUsers(pageable);
            adminController.getUserById(TEST_USER_ID);
            adminController.createUser(userCreateRequest);
            adminController.updateUser(TEST_USER_ID, userUpdateRequest);
            adminController.deleteUser(TEST_USER_ID);

            // then
            then(userService).should().getAllUsers(pageable);
            then(userService).should().getUserById(TEST_USER_ID);
            then(userService).should().createUser(userCreateRequest);
            then(userService).should().updateUser(TEST_USER_ID, userUpdateRequest);
            then(userService).should().deleteUser(TEST_USER_ID);
        }
    }
}