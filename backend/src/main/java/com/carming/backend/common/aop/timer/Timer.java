package com.carming.backend.common.aop.timer;

import org.aspectj.lang.ProceedingJoinPoint;

public interface Timer {
    Long start();

    Long getElapsedTime(Long startTime);
}
