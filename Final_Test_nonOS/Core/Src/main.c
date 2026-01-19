#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/*
ddddddddddddddddddddddddddddddddddddddddd
faiowejfiuhiuashdfuihiuoqwfuiohawfargiuahfiuohaweuif

*/

char tx_buffer[32]; // luu chuoi ky tu tra ve nhiet do
char rx_char; // ky tu stm32 nhan duoc

// gpio configuration
void GPIO_Init(void){
    // enable clock gpiob and gpiod
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    // PB6 (TX), PB7 (RX) -> alternate function
    GPIOB->MODER &= ~((0b11 << (6 * 2)) | (0b11 << (7 * 2)));
    GPIOB->MODER |=  ((0b10 << (6 * 2)) | (0b10 << (7 * 2)));

    // alternate function mode AF7 0111 for PB6 and PB7
    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |=  ((0x7 << (6 * 4)) | (0x7 << (7 * 4)));

    // PD12–PD15 → Output (on/off led)
    GPIOD->MODER &= ~(0xFF << (12 * 2));
    GPIOD->MODER |=  (0x55 << (12 * 2));
}

// usart1 configuration
void USART1_Init(void){
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Baudrate = 9600 ;  f = 16MHz
    USART1->BRR = 0x683;

    USART1->CR1 &= ~USART_CR1_M;     // 8 bit
    USART1->CR1 &= ~USART_CR1_PCE;   // No parity
    USART1->CR2 &= ~USART_CR2_STOP;  // 1 stop bit

    USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART1_IRQn);
}

// adc configuration
void ADC1_Init(void){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC->CCR |= ADC_CCR_TSVREFE;       // Enable temp sensor
    ADC1->SQR3 = 16;                   // Channel 16 = Temp sensor
    ADC1->SMPR1 |= (0b111 << 18);      // Sample time 480 cycles
    ADC1->CR2 |= ADC_CR2_ADON;         // Enable ADC1
}

// timer 2 configuration
void Timer2_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 16000 - 1;             // 1ms tick
    TIM2->ARR = 1000 - 1;              // 1s interval
    TIM2->DIER |= TIM_DIER_UIE;        // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN;          // Start timer
    NVIC_EnableIRQ(TIM2_IRQn);
}

// send char from stm32
void USART_SendChar(char c){
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
    while (!(USART1->SR & USART_SR_TC));
}

// send string from stm32
void USART_SendString(char* str){
    while (*str){
        USART_SendChar(*str++);
    }
}

// ===================update firmware ==============================================
//#if 0

#define FLASH_INTERFACE_BASE_ADDR  0x40023C00

__attribute__((section(".Function_in_RAM"))) void flash_erase_sector(int sec_num)
{
    uint32_t* FLASH_SR = (uint32_t*)(FLASH_INTERFACE_BASE_ADDR + 0x0C);
    uint32_t* FLASH_CR = (uint32_t*)(FLASH_INTERFACE_BASE_ADDR + 0x10);

    // Nếu Flash đang bị khóa thì mở khóa
    if (((*FLASH_CR) >> 31) == 1)
    {
        // Unlock sequence
        uint32_t* FLASH_KEYR = (uint32_t*)(FLASH_INTERFACE_BASE_ADDR + 0x04);
        *FLASH_KEYR = 0x45670123;
        *FLASH_KEYR = 0xCDEF89AB;
    }

    // Nếu sector vượt quá 7 thì không hợp lệ (trên STM32F411 chỉ có 8 sector)
    if (sec_num > 7)
        return;

    // 1. Chờ không còn thao tác Flash nào đang diễn ra
    while (((*FLASH_SR >> 16) & 1) == 1);

    // 2. Thiết lập bit SER (Sector Erase) và chọn sector cần xóa
    *FLASH_CR |= (1 << 1);              // SER = 1
    *FLASH_CR |= (sec_num << 3);        // SNB[3:0] = sector number

    // 3. Thiết lập bit STRT để bắt đầu xóa sector
    *FLASH_CR |= (1 << 16);             // STRT = 1

    // 4. Chờ cho đến khi bit BSY = 0 (hoàn thành)
    while (((*FLASH_SR >> 16) & 1) == 1);
}

//===============flash program =================
__attribute__((section(".Function_in_RAM"))) void flash_program(uint8_t* addr, uint8_t val)
{
    uint32_t* FLASH_SR = (uint32_t*)(FLASH_INTERFACE_BASE_ADDR + 0x0C);
    uint32_t* FLASH_CR = (uint32_t*)(FLASH_INTERFACE_BASE_ADDR + 0x10);

    // Nếu Flash đang bị khóa thì mở khóa
    if (((*FLASH_CR) >> 31) == 1)
    {
        // Unlock Flash bằng chuỗi khóa
        uint32_t* FLASH_KEYR = (uint32_t*)(FLASH_INTERFACE_BASE_ADDR + 0x04);
        *FLASH_KEYR = 0x45670123;   // KEY1
        *FLASH_KEYR = 0xCDEF89AB;   // KEY2
    }

    // 1. Đợi cho đến khi không còn hoạt động ghi nào đang diễn ra (BSY = 0)
    while (((*FLASH_SR >> 16) & 1) == 1);

    // 2. Set bit PG (program) trong thanh ghi FLASH_CR
    *FLASH_CR |= (1 << 0);

    // 3. Ghi dữ liệu vào địa chỉ flash
    *addr = val;

    // 4. Chờ đến khi việc ghi hoàn tất (BSY = 0)
    while (((*FLASH_SR >> 16) & 1) == 1);
}



#define UART1_BASE_ADDR 0x40011000
void uart_send(char data)
{
    uint32_t* UART_DR = (uint32_t*)(UART1_BASE_ADDR + 0x04);  // Thanh ghi dữ liệu
    *UART_DR = data;  // Gửi dữ liệu

    // Chờ cho đến khi việc truyền hoàn tất (bit 6 TXE trong SR = 1)
    uint32_t* UART_SR = (uint32_t*)(UART1_BASE_ADDR + 0x00);  // Thanh ghi trạng thái
    while (((*UART_SR >> 6) & 1) == 0);
}

char rx_buf[7760];
int rx_index = 0;

#define DMA2_BASE_ADDR  0x40026400

void DMA_Init()
{
    /*
        chọn DMA2 stream 2 channel 4 cho UART1 Rx
        - set địa chỉ người gửi
        - set địa chỉ người nhận
        - set số lượng data
    */

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    uint32_t* DMA_S2PAR  = (uint32_t*)(DMA2_BASE_ADDR + 0x18 + 0x18 * 2);
    uint32_t* DMA_S2M0AR = (uint32_t*)(DMA2_BASE_ADDR + 0x1C + 0x18 * 2);
    uint32_t* DMA_S2NDTR = (uint32_t*)(DMA2_BASE_ADDR + 0x14 + 0x18 * 2);
    uint32_t* DMA_S2CR   = (uint32_t*)(DMA2_BASE_ADDR + 0x10 + 0x18 * 2);

    *DMA_S2PAR  = 0x40011004;              // Địa chỉ thanh ghi UART1->DR (Data Register)
    *DMA_S2M0AR = (uint32_t)rx_buf;        // Bộ đệm nhận dữ liệu
    *DMA_S2NDTR = sizeof(rx_buf);          // Số lượng byte cần nhận

    *DMA_S2CR |= (4 << 25);                // Channel 4
    *DMA_S2CR |= (1 << 8);                 // Circular mode
    *DMA_S2CR |= (1 << 10);                // Memory increment mode

    /** dma send an interrupt signal*/
    *DMA_S2CR  |= 1 << 4;
    uint32_t* ISER1 = (uint32_t*)0xE000E104;
    *ISER1 |= 1 << (58 - 32);
    *DMA_S2CR |= 1 << 0;

}

char receive_new_fw = 0;
void DMA2_Stream2_IRQHandler(){
    __asm("NOP");
	uint32_t* DMA_LIFCR = (uint32_t*)(DMA2_BASE_ADDR + 0x08);
	*DMA_LIFCR |= 1 << 21;
	receive_new_fw = 1;
}

#include <string.h>
#include <stdarg.h>

void my_printf(char* str, ...)
{
    va_list list;
    va_start(list, str);

    char print_buf[128] = {0};                // Bộ đệm lưu chuỗi sau khi format
    vsprintf(print_buf, str, list);           // Format chuỗi với tham số biến

    int len = strlen(print_buf);
    for (int i = 0; i < len; i++)
    {
        uart_send(print_buf[i]);              // Gửi từng ký tự qua UART
    }

    va_end(list);
}

__attribute__((section(".Function_in_RAM"))) void update()
{
	// disable interrupt
	__asm("CPSID i");
	if (receive_new_fw == 1)
	    	{
	    		flash_erase_sector(0);
	    		for (int i = 0; i < sizeof(rx_buf); i++)
	    		{
	    			flash_program(0x08000000 + i, rx_buf[i]);
	    		}
	    		uint32_t* AIRCR = (uint32_t*)(0xE000ED0C);
	    		*AIRCR = (0x5FA << 16) | (1 << 2);

	    	}
}

//#endif
// ======================== ket thuc update firmware =====================

// read temperature
float Read_temperature(void){
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    uint16_t data_temp = ADC1->DR;

//    float V_sense =((data_temp * 3) / 4095);
//    float temperature = ((V_sense - 0.76) / 0.0025) + 25;
//    return temperature;

    float V_sense = (float)(data_temp * 3) / 4095;
    float temperature = (float)(((V_sense - 0.76) / 0.0025) + 25);
    return temperature;
}

// led control
void Led_on_off(uint8_t state){
    if (state){
        GPIOD->ODR |= (0xF << 12);  // led on
    } else {
        GPIOD->ODR &= ~(0xF << 12); // led off
    }
}

// receive signal and give feedback
void USART1_IRQHandler(void){
    if (USART1->SR & USART_SR_RXNE){
        rx_char = USART1->DR;

        if (rx_char == '1'){
            Led_on_off(1);
            USART_SendString("LED ON\r\n");
        }
        else if (rx_char == '0'){
            Led_on_off(0);
            USART_SendString("LED OFF\r\n");
        }
        else {
            USART_SendString("GUI LAI!!!\r\n");
        }
    }
}

// timer 2 gui nhiet do voi chu ky 1 giay
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF){
        TIM2->SR &= ~TIM_SR_UIF;

        // lay nhiet do tu Read_Temperature
        // in ra man hinh
        int temp = (int)Read_temperature();
        //
        sprintf(tx_buffer, "Nhiet do STM32: %d do C\r\n", temp);
        USART_SendString(tx_buffer);
    }
}

int main(void){
	HAL_Init();
    GPIO_Init();
    USART1_Init();
    ADC1_Init();
    Timer2_Init();
    my_printf("firmware 333333\n");

    while(1)
    {
    	if (receive_new_fw == 1)
    	{
    		update();
    	}
    }
    return 0;
}

