#define UART_RX_BUF_LEN (1 << 8)
#define UART_DMA DMA1
#define UART_RX_BUF_MASK (UART_RX_BUF_LEN - 1)

void usart_init(void);
int usart_tx(const char* str, const int len);
void usart_update_nl(void);
int usart_readline(char* str, const int len);
bool usart_is_tx_busy(void);
