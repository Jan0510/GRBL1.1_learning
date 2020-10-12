#include "grbl.h"

#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float)

//从字符串的某一位开始读取一个浮点数(checked)
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)                  
{
// 尝试用atof()实现
	/*========================================================================
	char *ptr = line + *char_counter;
	unsigned char c;
	bool isnegative;
	uint32_t intval;
	int8_t exp;
	uint8_t ndigit;
	bool isdecimal;
	float fval;
	
	// Grab first character and increment pointer. No spaces assumed in line.
	c = *ptr++;
  
	// Capture initial positive/minus character
	isnegative = false;
	if (c == '-') 
	{
		isnegative = true;
		c = *ptr++;
	} 
	else if (c == '+') 
	{
		c = *ptr++;
	}
  
	// Extract number into fast integer. Track decimal in terms of exponent value.
	intval = 0;
	exp = 0;
	ndigit = 0;
	isdecimal = false;
  
	while(1) 
	{
		c -= '0';
		if (c <= 9) 
		{
			ndigit++;
			if (ndigit <= MAX_INT_DIGITS) 
			{
				if (isdecimal) 
				{ 
					exp--; 
				}
			intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
		} 
		else 
		{
			if (!(isdecimal)) 
			{ 
				exp++; // Drop overflow digits
			}  
		}
		} else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) 
		{
			isdecimal = true;
		} 
		else 
		{
			break;
		}
		c = *ptr++;
	}
  
	// Return if no digits have been read.
	if (!ndigit) 
	{ 
		return(false);
	};
  
	// Convert integer into floating point.
	fval = (float)intval;
  
	// Apply decimal. Should perform no more than two floating point multiplications for the
	// expected range of E0 to E-4.
	if (fval != 0) 
	{
		while (exp <= -2) 
		{
			fval *= (float)0.01; 
			exp += (int8_t)2;
		}
		if (exp < 0) 
		{ 
			fval *= (float)0.1; 
		} 
		else if (exp > 0) 
		{
			do {
				fval *= (float)10.0;
			} while (--exp > 0);
		} 
	}

	// Assign floating point value with correct sign.    
	if (isnegative) 
	{
		*float_ptr = -fval;
	} 
	else 
	{
		*float_ptr = fval;
	}

	*char_counter = ptr - line - 1; // Set char_counter to next statement
  
	return(true);
	========================================================================*/
	
	
	uint8_t n= *char_counter;
	
	char c= *(line + n *sizeof(char));
	if(c == '+' || c == '-' || c== '.'|| isdigit(c))
	{	
		if(c == '0') //防止0x
		{
			char x = *(line + (n+1) *sizeof(char));
			if(x=='x' || x=='X')
			{
				*float_ptr=0;
				*char_counter=n+1;
				return true;			
			}
		}
		*float_ptr = atof(line + n *sizeof(char));           //ASIIC转浮点函数
		while(c == '+' || c == '-' || c== '.'|| isdigit(c))
		{
			n++;
			c=line[n];
		}
		*char_counter = n;
		return true;
	}
	else
	{
		return false;
	}
	
}


// Simple hypotenuse computation function.
float hypot_f(float x, float y) { return(sqrt(x*x + y*y)); }


