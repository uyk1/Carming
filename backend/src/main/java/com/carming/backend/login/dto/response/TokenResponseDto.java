package com.carming.backend.login.dto.response;

import lombok.Data;
import lombok.NoArgsConstructor;

@NoArgsConstructor
@Data
public class TokenResponseDto {

    private String tokenType;

    private String accessToken;

    public TokenResponseDto(String tokenType, String accessToken) {
        this.tokenType = tokenType;
        this.accessToken = accessToken;
    }
}
