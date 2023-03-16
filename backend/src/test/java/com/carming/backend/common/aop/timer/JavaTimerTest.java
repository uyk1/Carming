package com.carming.backend.common.aop.timer;

import org.assertj.core.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class JavaTimerTest {

    @Test
    @DisplayName("자바 기반 타이머 시간 측정: 5초")
    void javaTimerTest() throws InterruptedException {
        Timer timer = new JavaTimer();

        Long startTime = timer.start();
        Thread.sleep(5000);
        Long elapsedTime = timer.getElapsedTime(startTime);

        Assertions.assertThat(elapsedTime).isGreaterThanOrEqualTo(5000);
    }
}