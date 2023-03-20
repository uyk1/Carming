package com.carming.backend.member.controller;

import com.carming.backend.member.dto.request.PhoneNumberDto;
import com.carming.backend.member.service.ValidNumbersService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RequiredArgsConstructor
@RequestMapping("/api/valid-number")
@RestController
public class ValidNumberController {

    private final ValidNumbersService validService;

    @PostMapping
    public ResponseEntity<Void> makeValidNumber(@RequestBody PhoneNumberDto request) {
        validService.createValidNumbers(request);
        return new ResponseEntity<>(HttpStatus.OK);
    }
}
