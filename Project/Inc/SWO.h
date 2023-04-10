#pragma once

#include <stddef.h>
#include <stdint.h>

#define USART_DEBUG

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef SWO_DEBUG
    void PrintChar(char const c, uint8_t const portNumber);
    void PrintString(char const *s, uint8_t const portNumber);
    void PrintDefault(char const *str);
    void PrintDefaultN(char const *str, size_t const len);
#elif defined(USART_DEBUG)
    void PrintChar(char const c);
    void PrintString(char const *s);
    void PrintDefaultN(char const *str, size_t const len);
#endif

#ifdef __cplusplus
}
#endif
