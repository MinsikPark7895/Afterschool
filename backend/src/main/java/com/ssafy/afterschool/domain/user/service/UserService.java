package com.ssafy.afterschool.domain.user.service;

import com.ssafy.afterschool.domain.user.dto.UserCreateRequest;
import com.ssafy.afterschool.domain.user.dto.UserInfoResponse;
import com.ssafy.afterschool.domain.user.dto.UserUpdateRequest;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;

public interface UserService {

    UserInfoResponse createUser(UserCreateRequest request);

    UserInfoResponse getUserById(Long userId);

    Page<UserInfoResponse> getAllUsers(Pageable pageable);

    UserInfoResponse updateUser(Long userId, UserUpdateRequest request);

    void deleteUser(Long userId);

    UserInfoResponse getMyProfile(Long userId);
}