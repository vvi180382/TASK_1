	// Прием массива данных по заданному протоколу 
	// На примере UART в контроллере STM32F030K6T6
		
	#include "stm32f0xx.h"                    
	#include "stdlib.h"
	#include "stdbool.h"
	
	#define PRE_1_4				0x01020304                           //преамбола
	#define F_tact_USART		48000000							//частота тактирования UART
	#define F_tact_TIM			48000000							//частота тактирования TIM
	#define USART_bod			9600								//скорость передачи по UART
	#define Poly				0x31								//делитель CRC
	
	static uint32_t PREAM;                                     //переменная для заполнения данных и сравнения с преамболой
	static uint8_t DATA[256+7];                              //массив данных с преамболой и сервисной информацией
	static uint16_t N_byte;                                    //номер считанного байта
	static _Bool FLAG_recive, FLAG_error;
	
	void USART_Init(void);									//инициализация периферии (UART)
	void USART1_IRQHandler(void);                            //прерывание по приему очередного байта по UART
	_Bool CRC_Control(uint8_t *DATA, uint8_t CRC_byte);      //контроль правильности приема информации DATA с контрольным байтом CRC_byte
	void TIM14_Init(void);
	void TIM14_IRQHandler(void);

	void USART_Init(void)				//инициализация порта UART
	{
		RCC->APB2ENR |= RCC_AHBENR_GPIOAEN;									//включение тактирования выводов под UART					
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;											//включение тактирования UART
		GPIOA->MODER |= GPIO_MODER_MODER9_0;                         //А9 на выход для ТХ
		GPIOA->MODER &=~ GPIO_MODER_MODER10_Msk;					//А10 на вход для ТХ
		GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10;       //альтернативные ф-ции пинов для UART  А9-RX, A10-TX
		GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1;					//скорость пина ТХ 
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;                       //подтягивание к "+" RX
		USART1->BRR = (F_tact_USART / USART_bod)<<USART_BRR_DIV_MANTISSA_Pos;    //скорость шины
		USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;					//разрешение прерывания по приему, вкл шины на прием
		NVIC_EnableIRQ (USART1_IRQn);											// вкл прерывания от UART
		NVIC_SetPriority(USART1_IRQn, 9);			//приоритет прерывания UART выше, чем у TIM14, чтобы таймер не сбрасывал N_byte раньше окончания функции приема байта (для корректной работы сброса N_byte)		
	}
	
	void TIM14_Init(void)                  //инициализация TIM14 для контроля времени приема сообщения
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  //вкл тактирования TIM14
		TIM14->PSC = (F_tact_TIM/1000)-1;	//предделитель до 1мс
		TIM14->ARR = 1000;                    //предзаполнение 1000  (мс) - 1с
		TIM14->DIER = TIM_DIER_UIE;         //прерывание по переполнению
		TIM14->CR1 = TIM_CR1_OPM;            //вкл таймер в режим одиночного счета
		NVIC_EnableIRQ(TIM14_IRQn);         //вкл прерывания от TIM14
		NVIC_SetPriority(TIM14_IRQn, 10);   //приоритет прерывания TIM14 ниже, чем UART, чтобы таймер не сбрасывал N_byte раньше окончания функции приема байта (для корректной работы сброса N_byte)
	}
	
	void TIM14_IRQHandler (void)
	{
		N_byte=0;						//по достижении 1с переход в режим ожидания преамболы (сброс счетчика байт)
		TIM14->SR = 0;                                   //сброс флагов прерываний
		FLAG_error = 1;					//флаг ошибки взводим
	}
	
	
	void USART1_IRQHandler(void)                                         //процедура по прерыванию по приему очередного байта
	{
		if (USART1->ISR & USART_ISR_RXNE)    {            				 //если есть принятый байт
			if (N_byte == 0) {
				PREAM = PREAM<<8;											//переменная для хранения первых 4х байт, смещая данные, заполняем по 8бит каждый цикл
				PREAM |= (char) USART1->RDR;
				if (PREAM == PRE_1_4) { 									//если первые 4 байта соответствуют преамболе, переходим к приему байта №4, сбрасываем флаг наличия принятой информации
					N_byte=4; 
					FLAG_recive=0;
					TIM14->CR1 |= TIM_CR1_CEN;							//перезапускаем таймер 14 на 1000мс 
				}
			}			
			else {
				if (N_byte == (4+3+DATA[4]+1))  {                       //если все байты приняты (счетчик достиг КОЛ-ВО_БАЙТ + 7 + 1), возвращаем счетчик байт в позицию 0, считываем контрольный байт, запускаем проверку CRC
					N_byte = 0;
					if (CRC_Control(DATA, (char) USART1->RDR)) {FLAG_recive=1; FLAG_error=0;} else {FLAG_error=1;}
				}
				else {
					DATA[N_byte++] =  (char) USART1->RDR;                //иначе продолжаем заполнять массив
				}
			}
		}
	}
	
	_Bool CRC_Control(uint8_t *DATA_CRC, uint8_t CRC_byte)               //функция контроля данных   CRC-8
	{
		uint8_t CRC_ = 0xFF;
		uint16_t N = *(DATA+4) + 4 + 3;         //N= длинне массива данных с преамболой и сервисной информацией
		while (N--) {
			CRC_ ^= *(DATA_CRC++);
			for (uint8_t i=0; i<8; i++) {
				if (CRC_ & 0x80) { CRC_ =  (char) ((CRC_ << 1) ^ Poly); } else { CRC_ = (char) (CRC_ << 1); }
				}
		}
		if (CRC_ == CRC_byte) {return 1;} else {return 0;}   //1 - если данные приняты безошибочно
	}	
		  

	int main() 	{
		
		USART_Init();
		for (int i=0; i<4; i++) { DATA[i] = (char) ( PRE_1_4 >> ((3-i)*8) ); }       //предварительная запись преамболы в массив данных 
		__enable_irq();																
		
		while(1) {
			//Здесь в основной программе мы считываем флаги состояния (FLAG_recive, FLAG_error) и массив полученных данных DATA, где
			//DATA[0]..DATA[3] - преамбола
			//DATA[4] - длина сообщения
			//DATA[5] - адрес отправителя
			//DATA[6] - адрес приемника
			//DATA[7].. - массив данных
			//FLAG_recive - флаг наличия првильно принятого массива данных
			//FLAG_error - флаг ошибки во время приема последнего массива данных
		}
	}
