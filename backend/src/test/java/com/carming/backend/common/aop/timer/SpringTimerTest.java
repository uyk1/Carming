package com.carming.backend.common.aop.timer;

import org.assertj.core.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class SpringTimerTest {

    @Test
    @DisplayName("Spring 기반 타이머 측정: 5초")
    void SpringTimerTest() throws InterruptedException {
        Timer timer = new SpringTimer();

        Long startTime = timer.start();
        Thread.sleep(5000);
        Long elapsedTime = timer.getElapsedTime(startTime);

        Assertions.assertThat(elapsedTime).isGreaterThanOrEqualTo(5000);
    }

}