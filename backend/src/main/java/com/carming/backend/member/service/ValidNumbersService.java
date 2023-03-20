package com.carming.backend.member.service;

import com.carming.backend.externalApi.naver.sms.SmsFactory;
import com.carming.backend.externalApi.naver.sms.SmsResponse;
import com.carming.backend.member.domain.valid.ValidNumberFactory;
import com.carming.backend.member.dto.request.PhoneNumberDto;
import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.data.redis.core.ValueOperations;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.Duration;

@RequiredArgsConstructor
@Service
public class ValidNumbersService {

    private final StringRedisTemplate redisTemplate;

    private final SmsFactory smsFactory;

    @Transactional
    public String createValidNumbers(PhoneNumberDto request) {
        String phoneNumber = request.getPhoneNumber();
        String validNumbers = ValidNumberFactory.createValidNumbers().getValidNumbers();

        ValueOperations<String, String> operations = redisTemplate.opsForValue();
        operations.set(phoneNumber, validNumbers, Duration.ofMinutes(3));

        SmsResponse response = smsFactory.send(phoneNumber, validNumbers);

        return request.getPhoneNumber();
    }
}
