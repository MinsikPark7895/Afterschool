package com.ssafy.afterschool.domain.user.dto;

import com.ssafy.afterschool.global.constants.Role;
import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.validation.constraints.Size;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Schema(description = "사용자 수정 요청")
public class UserUpdateRequest {

    @Size(max = 50, message = "사용자명은 50자 이하여야 합니다")
    @Schema(description = "사용자명", example = "admin123")
    private String username;

    @Size(min = 8, max = 255, message = "비밀번호는 8자 이상 255자 이하여야 합니다")
    @Schema(description = "비밀번호", example = "newpassword123!")
    private String password;

    @Size(max = 100, message = "이름은 100자 이하여야 합니다")
    @Schema(description = "사용자 이름", example = "홍길동")
    private String name;

    @Schema(description = "사용자 권한", example = "ADMIN")
    private Role role;
}