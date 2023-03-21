package com.carming.backend.member.dto.request;

import com.carming.backend.member.domain.BirthInfo;
import com.carming.backend.member.domain.Member;
import lombok.Data;

@Data
public class MemberCreate {

    private String phoneNumber;

    private String password;

    private String passwordCheck;

    private String nickname;

    private String name;

    private BirthInfoDto birthInfo;

    private Boolean isValid;

    public Member toEntity() {
        return Member.builder()
                .phoneNumber(phoneNumber)
                .password(password)
                .nickname(nickname)
                .name(name)
                .birthInfo(birthInfo.toEntity())
                .build();
    }

    @Data
    private static class BirthInfoDto {
        private String birthYear;
        private String birthMonth;
        private String birthDay;

        public BirthInfo toEntity() {
            return new BirthInfo(birthYear, birthMonth, birthDay);
        }
    }
}
