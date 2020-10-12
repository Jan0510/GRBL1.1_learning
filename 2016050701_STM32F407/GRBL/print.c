#include "grbl.h"

// 串口打印一个字符串(checked)
void printString(const char *s) 
{
	while (*s)
	{	
		serial_write(*s++);
	}
}

// 串口打印一个字符串(checked)
void printPgmString(const char *s) 
{
	char c;
	int i=0;
	while((c=s[i])!='\0')
	{
		serial_write(c);
		i++;
	}
}

// 以digits位base进制形式打印一个u8整型数(checked)
void print_unsigned_int8(uint8_t n, uint8_t base, uint8_t digits) 
{
	unsigned char buf[10];
	uint8_t i = 0;
	for (; i < digits; i++) 
	{
		buf[i] = n % base ;
		n /= base;
	}
	for (; i > 0; i--)
	  serial_write('0' + buf[i - 1]);
}

// 以2进制打印u8(checked)
void print_uint8_base2(uint8_t n) 
{
	print_unsigned_int8(n,2,8);
}

// 串口打印10进制u8数(checked)
void print_uint8_base10(uint8_t n) 
{   
// 尝试用sprintf()简化
	/*======================
	uint8_t digits;
	if (n < 10) 
	{ 
		digits = 1; 
	} 
	else if (n < 100) 
	{ 
		digits = 2;
	}
	else 
	{ 
		digits = 3; 
	}
	print_unsigned_int8(n,10,digits);
	========================*/
	char s[10];
	sprintf(s,"%u",n);
	printString(s);
}
// 打印10进制u32数(checked)
void print_uint32_base10(uint32_t n)
{ 
// 尝试用sprintf()简化
	/*==========================
	unsigned char buf[10]; 
	uint8_t i;
	if (n == 0) 
	{
		serial_write('0');
		return;
	} 
	i = 0;
	while (n > 0) 
	{
		buf[i++] = n % 10;
		n /= 10;
	}

	for (; i > 0; i--)
	serial_write('0' + buf[i-1]);
	==========================*/
	char s[20];
	sprintf(s,"%u",n);
	printString(s);
}
// 打印长整型数(checked)
void printInteger(long n)
{
// 尝试用sprintf()简化	
	/*===========================
	if (n < 0) 
	{
		serial_write('-');
		print_uint32_base10(-n);
	} 
	else 
	{
		print_uint32_base10(n);
	}
	===========================*/
	char s[20];
	sprintf(s,"%ld",n);
	printString(s);
}
// 打印指定小数位的浮点数(checked)
void printFloat(float n, uint8_t decimal_places)
{
// 尝试用sprintf()简化
	/*=====================================================================
	uint8_t decimals;
	unsigned char buf[10]; 
	uint8_t i;
	uint32_t a;
	
	if (n < 0) 
	{
		serial_write('-');
		n = -n;
	}
	decimals = decimal_places;
	while (decimals >= 2) // Quickly convert values expected to be E0 to E-4.
	{ 
		n *= 100;
		decimals -= 2;
	}
	if (decimals) { n *= 10; }
	n += (float)0.5;
	
	// Generate digits backwards and store in string.
	i = 0;
	a = (long)n; 
	buf[decimal_places] = '.'; // Place decimal point, even if decimal places are zero.
	while(a > 0) 
	{
		if (i == decimal_places) { i++; } // Skip decimal point location
		buf[i++] = (a % 10) + '0'; // Get digit
		a /= 10;
	}
	while (i < decimal_places) 
	{ 
		buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
	}
	if (i == decimal_places) // Fill in leading zero, if needed.
	{ 
		i++;
		buf[i++] = '0'; 
	}   
  
	// Print the generated string.
	for (; i > 0; i--)
    serial_write(buf[i-1]);
	========================================================================*/
	char s[20];
	switch(decimal_places)
	{
		case 0:
			sprintf(s,"%.0f",n);
			break;
		case 1:
			sprintf(s,"%.1f",n);
			break;
		case 2:
			sprintf(s,"%.2f",n);
			break;
		case 3:
			sprintf(s,"%.3f",n);
			break;
		case 4:
			sprintf(s,"%.4f",n);
			break;
		case 5:
			sprintf(s,"%.5f",n);
			break;
		default:
			sprintf(s,"%.6f",n);
			break;		
	}
	printString(s);
}

// 打印当前坐标值(checked)

void printFloat_CoordValue(float n) 
{ 
	if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) 
	{ 
		printFloat(n*(float)INCH_PER_MM,N_DECIMAL_COORDVALUE_INCH);
	} 
	else 
	{
		printFloat(n,N_DECIMAL_COORDVALUE_MM);
	}
}

// 打印当前进给速度(checked)
void printFloat_RateValue(float n) 
{ 
	if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) 
	{
		printFloat(n*(float)INCH_PER_MM,N_DECIMAL_RATEVALUE_INCH);
	} 
	else 
	{
		printFloat(n,N_DECIMAL_RATEVALUE_MM);
	}	
}

// 打印设置值(checked)
void printFloat_SettingValue(float n) 
{ 	
	printFloat(n,N_DECIMAL_SETTINGVALUE); 
}


