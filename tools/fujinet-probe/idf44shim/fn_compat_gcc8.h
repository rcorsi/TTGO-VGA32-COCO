#pragma once
// Probe-only compat for building FujiNet (GCC 12 / C++20 / IDF 5) with GCC 8.4 / IDF 4.4.
#ifdef __cplusplus
#include <type_traits>
namespace std {
template <class D, class B>
concept derived_from = is_base_of_v<B, D> && is_convertible_v<const volatile D*, const volatile B*>;
}
#endif
#include "driver/uart.h"
#ifndef UART_SCLK_DEFAULT
#define UART_SCLK_DEFAULT UART_SCLK_APB
#endif
