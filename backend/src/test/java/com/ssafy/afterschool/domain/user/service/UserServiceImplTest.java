package com.ssafy.afterschool.domain.user.service;

import com.ssafy.afterschool.domain.user.dto.UserCreateRequest;
import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.dto.UserUpdateRequest;
import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.domain.user.repository.UserRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.constants.Role;
import com.ssafy.afterschool.global.exception.ApiException;
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
import org.springframework.security.crypto.password.PasswordEncoder;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.BDDMockito.*;

@ExtendWith(MockitoExtension.class)
@DisplayName("UserServiceImpl 단위 테스트")
class UserServiceImplTest {

    @Mock
    private UserRepository userRepository;

    @Mock
    private PasswordEncoder passwordEncoder;

    @InjectMocks
    private UserServiceImpl userService;

    private static final Long TEST_USER_ID = 1L;
    private static final String TEST_USERNAME = "testuser";
    private static final String TEST_PASSWORD = "password123";
    private static final String TEST_ENCODED_PASSWORD = "encodedPassword123";
    private static final String TEST_NAME = "Test User";
    private static final String NEW_USERNAME = "newuser";
    private static final String NEW_PASSWORD = "newpassword123";
    private static final String NEW_NAME = "New User";

    private User testUser;
    private UserCreateRequest createRequest;
    private UserUpdateRequest updateRequest;

    @BeforeEach
    void setUp() {
        testUser = User.builder()
                .id(TEST_USER_ID)
                .username(TEST_USERNAME)
                .password(TEST_ENCODED_PASSWORD)
                .name(TEST_NAME)
                .role(Role.OPERATOR)
                .isDeleted(false)
                .createdAt(LocalDateTime.now())
                .lastLoginAt(LocalDateTime.now())
                .build();

        createRequest = new UserCreateRequest(TEST_USERNAME, TEST_PASSWORD, TEST_NAME, Role.OPERATOR);

        updateRequest = new UserUpdateRequest(NEW_USERNAME, NEW_PASSWORD, NEW_NAME, Role.ADMIN);
    }

    @Nested
    @DisplayName("사용자 생성 테스트")
    class CreateUserTest {

        @Test
        @DisplayName("유효한 정보로 사용자 생성이 성공한다")
        void shouldCreateUserSuccessfullyWithValidData() {
            // given
            given(userRepository.existsByUsernameAndIsDeletedFalse(TEST_USERNAME))
                    .willReturn(false);
            given(passwordEncoder.encode(TEST_PASSWORD))
                    .willReturn(TEST_ENCODED_PASSWORD);
            given(userRepository.save(any(User.class)))
                    .willReturn(testUser);

            // when
            UserInfoResponse response = userService.createUser(createRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.getName()).isEqualTo(TEST_NAME);
                        assertThat(r.getRole()).isEqualTo(Role.OPERATOR);
                    });

            then(userRepository).should().existsByUsernameAndIsDeletedFalse(TEST_USERNAME);
            then(passwordEncoder).should().encode(TEST_PASSWORD);
            then(userRepository).should().save(any(User.class));
        }

        @Test
        @DisplayName("중복된 사용자명으로 생성 시 예외가 발생한다")
        void shouldThrowExceptionWhenUsernameAlreadyExists() {
            // given
            given(userRepository.existsByUsernameAndIsDeletedFalse(TEST_USERNAME))
                    .willReturn(true);

            // when & then
            assertThatThrownBy(() -> userService.createUser(createRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USERNAME_ALREADY_EXISTS);
                    });

            then(passwordEncoder).shouldHaveNoInteractions();
            then(userRepository).should(never()).save(any(User.class));
        }
    }

    @Nested
    @DisplayName("사용자 조회 테스트")
    class GetUserTest {

        @Test
        @DisplayName("ID로 사용자 조회가 성공한다")
        void shouldGetUserByIdSuccessfully() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));

            // when
            UserInfoResponse response = userService.getUserById(TEST_USER_ID);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.getName()).isEqualTo(TEST_NAME);
                        assertThat(r.getRole()).isEqualTo(Role.OPERATOR);
                    });

            then(userRepository).should().findByIdAndIsDeletedFalse(TEST_USER_ID);
        }

        @Test
        @DisplayName("존재하지 않는 사용자 조회 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFound() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.empty());

            // when & then
            assertThatThrownBy(() -> userService.getUserById(TEST_USER_ID))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });
        }

        @Test
        @DisplayName("모든 사용자 목록 조회가 성공한다")
        void shouldGetAllUsersSuccessfully() {
            // given
            User user2 = User.builder()
                    .id(2L)
                    .username("user2")
                    .password("password")
                    .name("User 2")
                    .role(Role.ADMIN)
                    .isDeleted(false)
                    .build();

            List<User> users = List.of(testUser, user2);
            Page<User> userPage = new PageImpl<>(users, PageRequest.of(0, 20), 2);

            given(userRepository.findByIsDeletedFalse(any(Pageable.class)))
                    .willReturn(userPage);

            // when
            Page<UserInfoResponse> response = userService.getAllUsers(PageRequest.of(0, 20));

            // then
            assertThat(response).isNotNull();
            assertThat(response.getContent()).hasSize(2);
            assertThat(response.getTotalElements()).isEqualTo(2);
            assertThat(response.getContent().get(0).getUserId()).isEqualTo(TEST_USER_ID);
            assertThat(response.getContent().get(1).getUserId()).isEqualTo(2L);

            then(userRepository).should().findByIsDeletedFalse(any(Pageable.class));
        }

        @Test
        @DisplayName("내 프로필 조회가 성공한다")
        void shouldGetMyProfileSuccessfully() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));

            // when
            UserInfoResponse response = userService.getMyProfile(TEST_USER_ID);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(TEST_USERNAME);
                        assertThat(r.getName()).isEqualTo(TEST_NAME);
                        assertThat(r.getRole()).isEqualTo(Role.OPERATOR);
                    });

            then(userRepository).should().findByIdAndIsDeletedFalse(TEST_USER_ID);
        }
    }

    @Nested
    @DisplayName("사용자 수정 테스트")
    class UpdateUserTest {

        @Test
        @DisplayName("모든 필드 수정이 성공한다")
        void shouldUpdateAllFieldsSuccessfully() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));
            given(userRepository.existsByUsernameAndIsDeletedFalse(NEW_USERNAME))
                    .willReturn(false);
            given(passwordEncoder.encode(NEW_PASSWORD))
                    .willReturn("newEncodedPassword");

            // when
            UserInfoResponse response = userService.updateUser(TEST_USER_ID, updateRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(NEW_USERNAME);
                        assertThat(r.getName()).isEqualTo(NEW_NAME);
                        assertThat(r.getRole()).isEqualTo(Role.ADMIN);
                    });

            then(userRepository).should().findByIdAndIsDeletedFalse(TEST_USER_ID);
            then(userRepository).should().existsByUsernameAndIsDeletedFalse(NEW_USERNAME);
            then(passwordEncoder).should().encode(NEW_PASSWORD);
        }

        @Test
        @DisplayName("일부 필드만 수정이 성공한다")
        void shouldUpdatePartialFieldsSuccessfully() {
            // given
            UserUpdateRequest partialUpdateRequest = new UserUpdateRequest(null, null, NEW_NAME, null);

            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));

            // when
            UserInfoResponse response = userService.updateUser(TEST_USER_ID, partialUpdateRequest);

            // then
            assertThat(response)
                    .isNotNull()
                    .satisfies(r -> {
                        assertThat(r.getUserId()).isEqualTo(TEST_USER_ID);
                        assertThat(r.getUsername()).isEqualTo(TEST_USERNAME); // 기존값 유지
                        assertThat(r.getName()).isEqualTo(NEW_NAME); // 변경됨
                        assertThat(r.getRole()).isEqualTo(Role.OPERATOR); // 기존값 유지
                    });

            then(userRepository).should().findByIdAndIsDeletedFalse(TEST_USER_ID);
            then(userRepository).should(never()).existsByUsernameAndIsDeletedFalse(anyString());
            then(passwordEncoder).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("동일한 사용자명으로 수정은 성공한다")
        void shouldUpdateWithSameUsernameSuccessfully() {
            // given
            UserUpdateRequest sameUsernameRequest = new UserUpdateRequest(TEST_USERNAME, null, NEW_NAME, null);

            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));

            // when
            UserInfoResponse response = userService.updateUser(TEST_USER_ID, sameUsernameRequest);

            // then
            assertThat(response).isNotNull();

            // 동일한 사용자명이므로 중복 체크를 하지 않음
            then(userRepository).should(never()).existsByUsernameAndIsDeletedFalse(anyString());
        }

        @Test
        @DisplayName("중복된 사용자명으로 수정 시 예외가 발생한다")
        void shouldThrowExceptionWhenUsernameAlreadyExists() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));
            given(userRepository.existsByUsernameAndIsDeletedFalse(NEW_USERNAME))
                    .willReturn(true);

            // when & then
            assertThatThrownBy(() -> userService.updateUser(TEST_USER_ID, updateRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USERNAME_ALREADY_EXISTS);
                    });

            then(passwordEncoder).shouldHaveNoInteractions();
        }

        @Test
        @DisplayName("존재하지 않는 사용자 수정 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFoundForUpdate() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.empty());

            // when & then
            assertThatThrownBy(() -> userService.updateUser(TEST_USER_ID, updateRequest))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });

            then(userRepository).should(never()).existsByUsernameAndIsDeletedFalse(anyString());
            then(passwordEncoder).shouldHaveNoInteractions();
        }
    }

    @Nested
    @DisplayName("사용자 삭제 테스트")
    class DeleteUserTest {

        @Test
        @DisplayName("사용자 소프트 삭제가 성공한다")
        void shouldDeleteUserSuccessfully() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.of(testUser));

            // when & then
            assertThatCode(() -> userService.deleteUser(TEST_USER_ID))
                    .doesNotThrowAnyException();

            then(userRepository).should().findByIdAndIsDeletedFalse(TEST_USER_ID);
        }

        @Test
        @DisplayName("존재하지 않는 사용자 삭제 시 예외가 발생한다")
        void shouldThrowExceptionWhenUserNotFoundForDelete() {
            // given
            given(userRepository.findByIdAndIsDeletedFalse(TEST_USER_ID))
                    .willReturn(Optional.empty());

            // when & then
            assertThatThrownBy(() -> userService.deleteUser(TEST_USER_ID))
                    .isInstanceOf(ApiException.class)
                    .satisfies(ex -> {
                        ApiException apiEx = (ApiException) ex;
                        assertThat(apiEx.getErrorCode()).isEqualTo(ErrorCode.USER_NOT_FOUND);
                    });
        }
    }
}