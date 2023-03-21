package com.carming.backend.externalApi.naver.sms;

import lombok.Data;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.PropertySource;

import java.util.List;

@PropertySource("classpath:naver-sms.properties")
@Data
public class Sms {

    @Value("${sms.type}")
    private String type;

    @Value("${sms.content-type}")
    private String contentType;

    @Value("${sms.country-code}")
    private String countryCode;
    @Value("${sms.from}")
    private String from;
    private String content;
    private List<Message> messages;

    public Sms(String to, String content) {
        this.content = formatContent(content);
        this.messages = List.of(new Message(to));
    }

    private String formatContent(String content) {
        return "[Carming] 인증번호: " + content + "\n" +
                "인증번호를 입력해주세요.";
    }


    @Data
    public static class Message {
        private String to;

        public Message(String to) {
            this.to = to;
        }
    }
}
