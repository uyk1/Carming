package com.carming.backend.order.controller;

import com.carming.backend.order.service.OrderService;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RequiredArgsConstructor
@RequestMapping("/api/order")
@RestController
public class OrderController {

    private final OrderService orderService;
}
