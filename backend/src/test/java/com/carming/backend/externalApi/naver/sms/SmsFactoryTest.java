package com.carming.backend.externalApi.naver.sms;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

@SpringBootTest
class SmsFactoryTest {

    @Autowired
    SmsFactory factory;

    @Test
    @DisplayName("")
    void makeSmsFactory() {

        factory.send("01051391314", "123456");

        System.out.println(factory.getServiceId());
        System.out.println(factory.getSecretKey());
        System.out.println(factory.getAccessKey());
        System.out.println(factory.getWebClient());
    }

}