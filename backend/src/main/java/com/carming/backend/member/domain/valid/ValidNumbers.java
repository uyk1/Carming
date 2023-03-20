package com.carming.backend.member.domain.valid;

import java.util.List;

public class ValidNumbers {

    private static final int VALID_NUMBERS_SIZE = 6;

    List<Integer> validNumbers;

    public ValidNumbers(List<Integer> numbers) {
        validSize(numbers);
        this.validNumbers = numbers;
    }

    public String getValidNumbers() {
        return validNumbers.stream()
                .map(String::valueOf)
                .reduce((x, y) -> x + y)
                .orElseThrow(() -> new RuntimeException("인증번호가 존재하지 않습니다."));
    }

    private void validSize(List<Integer> numbers) {
        if (numbers.size() != VALID_NUMBERS_SIZE) {
            throw new IllegalArgumentException("인증번호는 6자리여야 합니다.");
        }
    }
}
