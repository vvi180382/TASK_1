	// ����� ������� ������ �� ��������� ��������� 
	// �� ������� UART � ����������� STM32F030K6T6
		
	#include "stm32f0xx.h"                    
	#include "stdlib.h"
	#include "stdbool.h"
	
	#define PRE_1_4				0x01020304                           //���������
	#define F_tact_USART		48000000							//������� ������������ UART
	#define F_tact_TIM			48000000							//������� ������������ TIM
	#define USART_bod			9600								//�������� �������� �� UART
	#define Poly				0x31								//�������� CRC
	
	static uint32_t PREAM;                                     //���������� ��� ���������� ������ � ��������� � ����������
	static uint8_t DATA[256+7];                              //������ ������ � ���������� � ��������� �����������
	static uint16_t N_byte;                                    //����� ���������� �����
	static _Bool FLAG_recive, FLAG_error;
	
	void USART_Init(void);									//������������� ��������� (UART)
	void USART1_IRQHandler(void);                            //���������� �� ������ ���������� ����� �� UART
	_Bool CRC_Control(uint8_t *DATA, uint8_t CRC_byte);      //�������� ������������ ������ ���������� DATA � ����������� ������ CRC_byte
	void TIM14_Init(void);
	void TIM14_IRQHandler(void);

	void USART_Init(void)				//������������� ����� UART
	{
		RCC->APB2ENR |= RCC_AHBENR_GPIOAEN;									//��������� ������������ ������� ��� UART					
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;											//��������� ������������ UART
		GPIOA->MODER |= GPIO_MODER_MODER9_0;                         //�9 �� ����� ��� ��
		GPIOA->MODER &=~ GPIO_MODER_MODER10_Msk;					//�10 �� ���� ��� ��
		GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10;       //�������������� �-��� ����� ��� UART  �9-RX, A10-TX
		GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1;					//�������� ���� �� 
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;                       //������������ � "+" RX
		USART1->BRR = (F_tact_USART / USART_bod)<<USART_BRR_DIV_MANTISSA_Pos;    //�������� ����
		USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;					//���������� ���������� �� ������, ��� ���� �� �����
		NVIC_EnableIRQ (USART1_IRQn);											// ��� ���������� �� UART
		NVIC_SetPriority(USART1_IRQn, 9);			//��������� ���������� UART ����, ��� � TIM14, ����� ������ �� ��������� N_byte ������ ��������� ������� ������ ����� (��� ���������� ������ ������ N_byte)		
	}
	
	void TIM14_Init(void)                  //������������� TIM14 ��� �������� ������� ������ ���������
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  //��� ������������ TIM14
		TIM14->PSC = (F_tact_TIM/1000)-1;	//������������ �� 1��
		TIM14->ARR = 1000;                    //�������������� 1000  (��) - 1�
		TIM14->DIER = TIM_DIER_UIE;         //���������� �� ������������
		TIM14->CR1 = TIM_CR1_OPM;            //��� ������ � ����� ���������� �����
		NVIC_EnableIRQ(TIM14_IRQn);         //��� ���������� �� TIM14
		NVIC_SetPriority(TIM14_IRQn, 10);   //��������� ���������� TIM14 ����, ��� UART, ����� ������ �� ��������� N_byte ������ ��������� ������� ������ ����� (��� ���������� ������ ������ N_byte)
	}
	
	void TIM14_IRQHandler (void)
	{
		N_byte=0;						//�� ���������� 1� ������� � ����� �������� ��������� (����� �������� ����)
		TIM14->SR = 0;                                   //����� ������ ����������
		FLAG_error = 1;					//���� ������ �������
	}
	
	
	void USART1_IRQHandler(void)                                         //��������� �� ���������� �� ������ ���������� �����
	{
		if (USART1->ISR & USART_ISR_RXNE)    {            				 //���� ���� �������� ����
			if (N_byte == 0) {
				PREAM = PREAM<<8;											//���������� ��� �������� ������ 4� ����, ������ ������, ��������� �� 8��� ������ ����
				PREAM |= (char) USART1->RDR;
				if (PREAM == PRE_1_4) { 									//���� ������ 4 ����� ������������� ���������, ��������� � ������ ����� �4, ���������� ���� ������� �������� ����������
					N_byte=4; 
					FLAG_recive=0;
					TIM14->CR1 |= TIM_CR1_CEN;							//������������� ������ 14 �� 1000�� 
				}
			}			
			else {
				if (N_byte == (4+3+DATA[4]+1))  {                       //���� ��� ����� ������� (������� ������ ���-��_���� + 7 + 1), ���������� ������� ���� � ������� 0, ��������� ����������� ����, ��������� �������� CRC
					N_byte = 0;
					if (CRC_Control(DATA, (char) USART1->RDR)) {FLAG_recive=1; FLAG_error=0;} else {FLAG_error=1;}
				}
				else {
					DATA[N_byte++] =  (char) USART1->RDR;                //����� ���������� ��������� ������
				}
			}
		}
	}
	
	_Bool CRC_Control(uint8_t *DATA_CRC, uint8_t CRC_byte)               //������� �������� ������   CRC-8
	{
		uint8_t CRC_ = 0xFF;
		uint16_t N = *(DATA+4) + 4 + 3;         //N= ������ ������� ������ � ���������� � ��������� �����������
		while (N--) {
			CRC_ ^= *(DATA_CRC++);
			for (uint8_t i=0; i<8; i++) {
				if (CRC_ & 0x80) { CRC_ =  (char) ((CRC_ << 1) ^ Poly); } else { CRC_ = (char) (CRC_ << 1); }
				}
		}
		if (CRC_ == CRC_byte) {return 1;} else {return 0;}   //1 - ���� ������ ������� �����������
	}	
		  

	int main() 	{
		
		USART_Init();
		for (int i=0; i<4; i++) { DATA[i] = (char) ( PRE_1_4 >> ((3-i)*8) ); }       //��������������� ������ ��������� � ������ ������ 
		__enable_irq();																
		
		while(1) {
			//����� � �������� ��������� �� ��������� ����� ��������� (FLAG_recive, FLAG_error) � ������ ���������� ������ DATA, ���
			//DATA[0]..DATA[3] - ���������
			//DATA[4] - ����� ���������
			//DATA[5] - ����� �����������
			//DATA[6] - ����� ���������
			//DATA[7].. - ������ ������
			//FLAG_recive - ���� ������� �������� ��������� ������� ������
			//FLAG_error - ���� ������ �� ����� ������ ���������� ������� ������
		}
	}
