/*
 * I2C_OLED_BMP180_Atmega8.c
 *
 * Created: 22-11-2023 21:27:37
 * Author : naval
 */ 
#define F_CPU	16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/deprecated.h>
#include <stdio.h>
#include <string.h>
#include "i2cnaval.c"
#include "i2c_oled_SSD1306.c"


#define BMP180_ADDR_WRITE			0xEE // WRITE address ON BMP180
#define BMP180_ADDR_READ			0xEF // READ  address ON BMP180
#define	BMP180_REG_CONTROL			0xF4
#define	BMP180_REG_RESULT			0xF6

#define	BMP180_COMMAND_TEMPERATURE	0x2E
#define	BMP180_COMMAND_PRESSURE0	0x34
#define	BMP180_COMMAND_PRESSURE1	0x74
#define	BMP180_COMMAND_PRESSURE2	0xB4
#define	BMP180_COMMAND_PRESSURE3	0xF4

unsigned char ac1[22],ac2[2],str[20];
short AC1,AC2,AC3,B1,B2,MB,MC,MD; // from datasheet ref.
unsigned short AC4,AC5,AC6,ut,pu;
double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2,T,X1, bz,c3,c4,b1; // from web reference
char _error;
double bmp_temp,bmp_pressure,altitude;

volatile int flag_1000ms=0;
volatile int flag_500ms=0;

void bmp180_init()
{
	i2c_init();
	i2c_start();
	i2c_write(0xee);  // address with write command
	i2c_write(0xaa);  // starting address for reading BMP180 EEPROM		
	_delay_ms(100);
	i2c_start();	// restart for read mode
	_delay_ms(10);
	i2c_write(0xef); // address with read mode on i2c bus
	ac1[0]=i2c_read(1); // after every ack address shift to next address in sequence
	ac1[1]=i2c_read(1);
	ac1[2]=i2c_read(1);
	ac1[3]=i2c_read(1);
	ac1[4]=i2c_read(1);
	ac1[5]=i2c_read(1);
	ac1[6]=i2c_read(1);
	ac1[7]=i2c_read(1);
	ac1[8]=i2c_read(1);
	ac1[9]=i2c_read(1);
	ac1[10]=i2c_read(1);
	ac1[11]=i2c_read(1);
	ac1[12]=i2c_read(1);
	ac1[13]=i2c_read(1);
	ac1[14]=i2c_read(1);
	ac1[15]=i2c_read(1);
	ac1[16]=i2c_read(1);
	ac1[17]=i2c_read(1);
	ac1[18]=i2c_read(1);
	ac1[19]=i2c_read(1);
	ac1[20]=i2c_read(1);
	ac1[21]=i2c_read(0);
	i2c_stop;
	
	AC1=(ac1[0]<<8)|ac1[1]; // bit shifting // 2 char(8 bit) converted to 16 bit by shifting and adding
	AC2=(ac1[2]<<8)|ac1[3]; // after shifting value is now in signed decimal data type
	AC3=(ac1[4]<<8)|ac1[5];
	AC4=(ac1[6]<<8)|ac1[7];
	AC5=(ac1[8]<<8)|ac1[9];
	AC6=(ac1[10]<<8)|ac1[11];
	B1=(ac1[12]<<8)|ac1[13];
	B2=(ac1[14]<<8)|ac1[15];
	MB=(ac1[16]<<8)|ac1[17];
	MC=(ac1[18]<<8)|ac1[19];
	MD=(ac1[20]<<8)|ac1[21];
	
	/*AC1=408;				// calibrated values from datasheet of BMP180
	AC2=-72;
	AC3=-14383;
	AC4=32741;
	AC5=32757;
	AC6=23153;
	B1=6190;
	B2=4;
	MB=-32768;
	MC=-8711;
	MD=2868;*/

	c5 = (pow(2,-15) / 160) * AC5; // math calculation from web // for temperature
	c6 = AC6;                                                    // for temperature
	mc = (pow(2,11) / pow(160,2)) * MC;                         // for temperature
	md = MD / 160.0;											 // for temperature
	
	c3 = 160.0 * pow(2,-15) * AC3;            // below values are for pressure and altitue measurement
	c4 = pow(10,-3) * pow(2,-15) * AC4;
	b1 = pow(160,2) * pow(2,-30) * B1;
	x0 = AC1;
	x1 = 160.0 * pow(2,-13) * AC2;
	x2 = pow(160,2) * pow(2,-25) * B2;
	y0 = c4 * pow(2,15);
	y1 = c4 * c3;
	y2 = c4 * b1;
	p0 = (3791.0 - 8.0) / 1600.0;
	p1 = 1.0 - 7357.0 * pow(2,-20);
	p2 = 3038.0 * 100.0 * pow(2,-36);
	
}

double get_temp()
{	
	// reading 16 bit raw value of ADC for temp on BMP180
	i2c_start();
	i2c_write(BMP180_ADDR_WRITE); // write mode with device address
	i2c_write(BMP180_REG_CONTROL); // control register
	i2c_write(BMP180_COMMAND_TEMPERATURE); // command for temperature ADC conversion
	i2c_stop();
	_delay_ms(10);
	
	i2c_start();
	i2c_write(BMP180_ADDR_WRITE);  // write mode with device address
	i2c_write(BMP180_REG_RESULT);  // goto result regsiter
	_delay_ms(10);
	i2c_start();
	i2c_write(BMP180_ADDR_READ); // read from result register
	ac2[0]=i2c_read(1);  // MSB read
	ac2[1]=i2c_read(0);  // LSB read
	
	i2c_stop();
	
	ut=(ac2[0]<<8)|ac2[1];  // bit shifting for adding 2 char to make 16 bit
	
	X1 = c5 * (ut - c6);
	T = X1 + ( mc / (X1 + md) );  // math calculations from web reference
	
	return T;

}

double getBMP180_pressure()
{	
	double s,x,y,z,P;
	
	// reading 16 bit raw value of ADC for temp on BMP180
	i2c_start();
	i2c_write(BMP180_ADDR_WRITE); // write mode with device address
	i2c_write(BMP180_REG_CONTROL); // control register
	i2c_write(BMP180_COMMAND_PRESSURE0); // command for pressure ADC conversion
	_delay_ms(50);
	i2c_stop();
	_delay_ms(10);
	
	i2c_start();
	i2c_write(BMP180_ADDR_WRITE);  // write mode with device address
	i2c_write(BMP180_REG_RESULT);  // goto result regsiter
	_delay_ms(10);
	i2c_start();
	i2c_write(BMP180_ADDR_READ); // read from result register
	ac2[0]=i2c_read(1);  // MSB read
	ac2[1]=i2c_read(1);  // LSB read
	ac2[2]=i2c_read(0);
	i2c_stop();
	
	pu=(ac2[0]<<8)|ac2[1]; //>>(8-0);  // bit shifting for adding 2 char to make 16 bit
	//pu=65500;
	//sprintf(str,"%u",pu);
	//uart_txstr(str);
	s = bmp_temp - 25.0;
	x = (x2 * pow(s,2)) + (x1 * s) + x0;
	y = (y2 * pow(s,2)) + (y1 * s) + y0;
	z = (pu - x) / y;
	P = (p2 * pow(z,2)) + (p1 * z) + p0;  // math calulations from web
	
	return P;

}


// Timer0 ISR for setting task flag

ISR(TIMER0_OVF_vect)
{
	static	int count_1000ms=0;
	static	int count_500ms=0;
	
	if(count_1000ms == 2000)
	{
		flag_1000ms = 1;
		count_1000ms = 0;
	}
	
	if(count_500ms == 500)
	{
		flag_500ms = 1;
		count_500ms = 0;
	}
	
	count_500ms++;
	count_1000ms++;
	TCNT0 = 240;		// COUNT FOR 1 MS
}

void timer0_init()
{
	TCCR0 = 0X05;			//	PRESCALAR AS 1024
	TCNT0 = 240;			//	COUNT FOR 1 MS
	TIMSK |= (1 << TOIE0);  //	Unmask Timer 0 overflow interrupt.
}

void get_bmp180_reading(void)
{
	bmp_temp=get_temp();
	_delay_ms(100);
	bmp_pressure=getBMP180_pressure();
	
	cbi(PORTB,0);
	sbi(PORTB,1);
}

int main(void)
{
    /* Replace with your application code */

  
  sbi(DDRB,0);
  sbi(DDRB,1);
  
  unsigned short a;
  float c=sizeof(a);
  
  lcd_init(LCD_DISP_ON);
  _delay_ms(10);
  lcd_puts("HelloWorld");
  _delay_ms(4000);
  lcd_clrscr();
   _delay_ms(10);
  

  bmp180_init();
   _delay_ms(10);
  
  timer0_init();
   _delay_ms(10);
  
  sei();			// global interrupt enable
  
  /* Replace with your application code */
  while (1)
  {
  if(flag_1000ms == 1)
  {
	  sbi(PORTB,0);
	  cbi(PORTB,1);
	  
	  get_bmp180_reading();
	
	  lcd_clrscr();
	   _delay_ms(10);
	  lcd_puts("TC=");
	  dtostrf(bmp_temp,2,1,str);
	  lcd_puts(str);
	   _delay_ms(10);
	  lcd_gotoxy(0,3);
	  lcd_puts("HG= ");
	  dtostrf(bmp_pressure,2,1,str);
	  lcd_puts(str); 
	   _delay_ms(10);
	  
	  altitude = 44330.0 * (1.0 - pow((bmp_pressure*100) / 101325.0, (1.0/5.255)));
	  lcd_gotoxy(0,6);
	  lcd_puts("Alt=");
	  dtostrf(altitude,2,1,str);
	  lcd_puts(str);
	  _delay_ms(10);
	  flag_1000ms = 0;
  }
  
  else if(flag_500ms == 1)
  {
	  
	  flag_500ms = 0;
  }
}
}

