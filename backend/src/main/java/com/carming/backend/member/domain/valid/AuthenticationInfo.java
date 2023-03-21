package com.carming.backend.member.domain.valid;

import lombok.Getter;

@Getter
public class AuthenticationInfo {

    String authNumbers;

    Boolean isAuthenticated;

    public AuthenticationInfo(String authNumbers, Boolean isAuthenticated) {
        this.authNumbers = authNumbers;
        this.isAuthenticated = isAuthenticated;
    }
}
