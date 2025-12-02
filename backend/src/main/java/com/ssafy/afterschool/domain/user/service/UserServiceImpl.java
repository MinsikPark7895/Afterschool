package com.ssafy.afterschool.domain.user.service;

import com.ssafy.afterschool.domain.user.dto.UserCreateRequest;
import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.dto.UserUpdateRequest;
import com.ssafy.afterschool.domain.user.entity.User;
import com.ssafy.afterschool.domain.user.repository.UserRepository;
import com.ssafy.afterschool.global.constants.ErrorCode;
import com.ssafy.afterschool.global.exception.ApiException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Slf4j
@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class UserServiceImpl implements UserService {

    private final UserRepository userRepository;
    private final PasswordEncoder passwordEncoder;

    @Override
    @Transactional
    public UserInfoResponse createUser(UserCreateRequest request) {
        if (userRepository.existsByUsernameAndIsDeletedFalse(request.getUsername())) {
            throw new ApiException(ErrorCode.USERNAME_ALREADY_EXISTS);
        }

        User user = User.builder()
                .username(request.getUsername())
                .password(passwordEncoder.encode(request.getPassword()))
                .name(request.getName())
                .role(request.getRole())
                .build();

        User savedUser = userRepository.save(user);
        return UserInfoResponse.from(savedUser);
    }

    @Override
    public UserInfoResponse getUserById(Long userId) {
        User user = findUserById(userId);
        return UserInfoResponse.from(user);
    }

    @Override
    public Page<UserInfoResponse> getAllUsers(Pageable pageable) {
        Page<User> users = userRepository.findByIsDeletedFalse(pageable);
        return users.map(UserInfoResponse::from);
    }

    @Override
    @Transactional
    public UserInfoResponse updateUser(Long userId, UserUpdateRequest request) {
        User user = findUserById(userId);

        if (request.getUsername() != null) {
            if (!request.getUsername().equals(user.getUsername()) &&
                userRepository.existsByUsernameAndIsDeletedFalse(request.getUsername())) {
                throw new ApiException(ErrorCode.USERNAME_ALREADY_EXISTS);
            }
            user.changeUsername(request.getUsername());
        }

        if (request.getPassword() != null) {
            user.changePassword(passwordEncoder.encode(request.getPassword()));
        }

        if (request.getName() != null) {
            user.changeName(request.getName());
        }

        if (request.getRole() != null) {
            user.changeRole(request.getRole());
        }

        return UserInfoResponse.from(user);
    }

    @Override
    @Transactional
    public void deleteUser(Long userId) {
        User user = findUserById(userId);
        user.markAsDeleted();
    }

    @Override
    public UserInfoResponse getMyProfile(Long userId) {
        User user = findUserById(userId);
        return UserInfoResponse.from(user);
    }

    private User findUserById(Long userId) {
        return userRepository.findByIdAndIsDeletedFalse(userId)
                .orElseThrow(() -> new ApiException(ErrorCode.USER_NOT_FOUND));
    }
}