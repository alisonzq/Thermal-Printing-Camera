#include "stc12c5a60s2.h"
#include "DP_Print_inc.h"

typedef unsigned char 	BYTE;
typedef unsigned char		WORD;
#define	FOSC	11059200L
#define	BAUD	9600

code unsigned char password[]={"20000508"};	// 此是密码，使用时向业务经理要正确密码
code unsigned char printTest[]={0xB4,0xEF,0xC6,0xD5,0xB4,0xF2,0xD3,0xA1,0x44,0x50};

BYTE busy;

void UARTInit(void)
{
	SCON = 0x50;
	TMOD = 0x20;
	TH1 = TL1 = -(FOSC/12/32/BAUD);
	TR1 = 1;
	ES = 1;
	EA = 1;
}

void UART_SendByte(unsigned char Send_Dat)
{
	while(busy);
	ACC = Send_Dat;
	busy = 1;
	SBUF = ACC;
}

void main(void)
{
	UARTInit();
	CheckPassWord(password);
	InitializePrint();
	SelChineseChar();
	Print_ASCII((unsigned char*)printTest,10);
	print_And_Line();
	while(1)
	{
	
	}
}

void Uart_isr() interrupt 4 using 1
{
	if(RI)
	{
		RI=0;
	}
	if(TI)
	{
		TI=0;
		busy=0;
	}
}