package com.carming.backend.member.domain;


import lombok.AccessLevel;
import lombok.NoArgsConstructor;

import javax.persistence.Column;
import javax.persistence.Embeddable;

@NoArgsConstructor(access = AccessLevel.PROTECTED)
@Embeddable
public class BirthInfo {

    @Column(name = "member_birth_year")
    private String birthYear;
    @Column(name = "member_birth_month")
    private String birthMonth;
    @Column(name = "member_birth_day")
    private String birthDay;

    public BirthInfo(String birthYear, String birthMonth, String birthDay) {
        this.birthYear = birthYear;
        this.birthMonth = birthMonth;
        this.birthDay = birthDay;
    }
}
