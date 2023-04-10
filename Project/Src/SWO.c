#include "SWO.h"
#include "main.h"

// extern UART_HandleTypeDef huart2;

#ifdef SWO_DEBUG
void PrintChar(char const c, uint8_t const portNumber)
{
    volatile int timeout;

    /* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
    /* check Trace Control Register if ITM trace is enabled*/
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0)
    {
        return; /* not enabled? */
    }
    /* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
    /* check Trace Enable Register if requested port is enabled */
    if ((ITM->TER & (1ul << portNumber)) == 0)
    {
        return; /* requested port not enabled? */
    }
    timeout = 5000; /* arbitrary timeout value */
    while (ITM->PORT[0].u32 == 0)
    {
        /* Wait until STIMx is ready, then send data */
        if (--timeout == 0)
        {
            return; /* not able to send */
        }
    }
    ITM->PORT[0].u8 = (uint8_t)c;
}

void PrintString(char const *s, uint8_t const portNumber)
{
    while (*s != '\0')
    {
        PrintChar(*s++, portNumber);
    }
}

void PrintDefault(char const *str)
{
    PrintString(str, 0);
}

void PrintDefaultN(char const *str, size_t const len)
{
    for (size_t i = 0; i < len; ++i)
    {
        PrintChar(str[i], 0);
    }
}

#elif defined(USART_DEBUG)

static USART_TypeDef *printUsart = USART2;

void PrintChar(char const c)
{
    while (LL_USART_IsActiveFlag_TXE(printUsart))
        ;
    LL_USART_TransmitData8(printUsart, c);
}
void PrintString(char const *s)
{
    for (int i = 0; i < strlen(s); i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(printUsart))
            ;
        LL_USART_TransmitData8(printUsart, s[i]);
    }
}
void PrintDefaultN(char const *str, size_t const len)
{
    for (int i = 0; i < len; i++)
    {
        // wait untill DR empty
        while (!LL_USART_IsActiveFlag_TXE(printUsart))
            ;
        LL_USART_TransmitData8(printUsart, str[i]);
    }
}
#endif