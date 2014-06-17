/************************************************************************************
 
 This experiment demostrates the application of a simple line follower robot. The 
 robot follows a white line over a black backround
 
 Connection Details:  	L-1 PA0		L-2 PA1
   						R-1 PA2		R-2	PA3
   						PWML OC5A	PWMR OC5B
*************************************************************************************/
/********************************************************************************
 Author:
        Abhishek Kabra
        Kapil Dubey
        Kanwal Prakash Singh
        Saurabh Bhola
		
		Group 9
		2012 Batch 
		CS308 course project
		

 Date: 18th April 2012
 
  Application example: Adaptive Cruise Control (ACC)

 Concepts covered:  ADC, LCD interfacing,zigbee, motion control based on sensor data

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 11059200
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/

/*********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"//included to support lcd prints

#define FCPU 11059200ul //defined here to make sure that program works properly

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned int time_required=1000;
unsigned char flag1 = 0;
unsigned char flag2 = 0;
//variables used importantly in the program
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Front_Sharp_Sensor=0;
unsigned char Front_IR_Sensor=0;
unsigned char Left_Sharp_Sensor=0;
unsigned char Right_Sharp_Sensor=0;

unsigned char data;
//unsigned char output_enable=PORTC & 0x80;
//unsigned char start_conv = PORTC & 0x40;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
int turn=-1;

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	buzzer_pin_config();	
}

// Timer 5 initialised in PWM mode for velocity control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:674.988Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionalit to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
//function to switch on the buzzer
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}
//function to switch off the buzzer
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}


void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}
//function to set the direction of motor in forward and start the motor
void forward (void) 
{
  motion_set (0x06);//0110
}
//function to stop the motor
void stop (void)
{
  motion_set (0x00);
}
//function to move the  bot in left direction 
void move_left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x04);
}
//function to move the  bot in right direction
void move_right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x02);
}
//function to initialize the zigbee configuration to recieve message from matlab
void uart0_init(void)
{
	 UCSR0B = 0x00; //disable while setting baud rate
	 UCSR0A = 0x00;
	 UCSR0C = 0x06;
	 UBRR0L = 0x5f; //set baud rate lo
	 UBRR0H = 0x00; //set baud rate hi
	 UCSR0B = 0x98;
}
//function that recieves signals sent from matlab which are obtained by using the camera
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

	UDR0 = data; 				//echo data back to PC
		if(data == 'l') //ASCII value of 7
		{
			turn=0;
		}

		if(data == 'r') //ASCII value of 9
		{
			turn=1;
		}

}


unsigned char data_array[8];
//function that print various port values at different position using print_sensor function
void print_it(void){
	   print_sensor(1,1,1);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,3);	//Prints Value of White Line Sensor3
		
	
		print_sensor(2,1,9);
		print_sensor(2,5,11);
		print_sensor(2,9,13);
}
//white line follower subroutine
void line_follower(unsigned char speed1, unsigned char speed2){
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	Front_Sharp_Sensor = ADC_Conversion(11);

	flag=0;

	if(Center_white_line <40){
		flag=1;
		velocity(speed2,speed2);
		forward();
	}
	if(Left_white_line > 40  && (flag==0)){
		flag=1;
		velocity(speed2,speed1);
	//	turn=1;//overtake from right
		forward();
	}
	if(Right_white_line > 40  && (flag ==0)){
		flag=1;
		velocity(speed1,speed2);
	//	turn=0;//overtake from left	
		forward();
	}
	if((Left_white_line>40) && (Right_white_line>40) && (Center_white_line>40)){
		stop();
	}
	if((Left_white_line<40) && (Right_white_line<40) && (Center_white_line<40)){
		forward();
	}
}
//overtaking subroutine
void overtaking(void){
	unsigned int time=0;
	//print_it();
	turn=0;
	if(turn==1){//move from right lane
		velocity(200,200);
		while(!(Center_white_line >40 && Right_white_line >40 && Left_white_line>40)){
			for(i=0;i<8;i++){
				data_array[i]=ADC_Conversion(i);
			}
			Left_white_line = data_array[3];	//Getting data of Left WL Sensor
			Center_white_line = data_array[2];  //Getting data of Center WL Sensor
			Right_white_line = data_array[1];	//Getting data of Right WL Sensor
		  	print_it();
			move_right();
			_delay_ms(50);
		}
		_delay_ms(50);
		stop();
		//forward();					 //Forward moving form left lane toward right lane
		Center_white_line=200;
		while(Center_white_line>40)	 //Forward Till White Line of Right Lane not detecsdted
		{
			buzzer_on();
			for(i=0;i<8;i++){
					data_array[i]=ADC_Conversion(i);
			}
			Left_white_line = data_array[3];	//Getting data of Left WL Sensor
			Center_white_line = data_array[2];  //Getting data of Center WL Sensor
			Right_white_line = data_array[1];	//Getting data of Right WL Sensor
		  	Front_Sharp_Sensor=ADC_Conversion(11);
		  	print_it();
	      	forward();
			_delay_ms(50);
		}
		buzzer_off();
		forward();	// Correction for white line detection
		_delay_ms(500);
		stop();
		print_it();
		while(!(Center_white_line <40 && Right_white_line <40)){
		  for(i=0;i<8;i++){
			data_array[i]=ADC_Conversion(i);
			}
			Left_white_line = data_array[3];	//Getting data of Left WL Sensor
			Center_white_line = data_array[2];  //Getting data of Center WL Sensor
			Right_white_line = data_array[1];	//Getting data of Right WL Sensorprint_it();
			move_left();
			_delay_ms(50);
		}
		stop();
		Left_Sharp_Sensor=ADC_Conversion(9);
		/*while(time<2*time_required)  //time required to overtake (time in units)
		{
		  time++;
		  line_follower(90,150);
		}*/
		while(Left_Sharp_Sensor>80){
			buzzer_on();
			print_sensor(1,1,9);
			line_follower(140,200);//check from left sensor
			Left_Sharp_Sensor=ADC_Conversion(9);
		}
		time=0;
		buzzer_off();
		/*while(time<2*time_required)  //time required to overtake (time in units)
		{
		  time++;
		  line_follower(90,150);
		}*/
		stop();
		velocity(200,200);
		while(!(Center_white_line >40 && Left_white_line >40 && Right_white_line >40)){
		    for(i=0;i<8;i++){
				data_array[i]=ADC_Conversion(i);
			}
		  	Left_white_line = data_array[3];	//Getting data of Left WL Sensor
		  	Center_white_line = data_array[2];  //Getting data of Center WL Sensor
		  	Right_white_line = data_array[1];	//Getting data of Right WL Sensor
		  	print_it();
		  	move_left();
	    	_delay_ms(50);
		}
		stop();
		forward();
		Center_white_line=200;
		while(Center_white_line>40)	 //Forward Till White Line of Right Lane not detected
		{
		    for(i=0;i<8;i++){
				data_array[i]=ADC_Conversion(i);
			}
		  	Left_white_line = data_array[3];	//Getting data of Left WL Sensor
		  	Center_white_line = data_array[2];  //Getting data of Center WL Sensor
		  	Right_white_line = data_array[1];
		    Front_Sharp_Sensor=ADC_Conversion(11);
		    print_it();
	        forward();
			_delay_ms(50);
		}
		forward();	// Correction for white line detection
		_delay_ms(500);
		stop();
		while(!(Center_white_line <40 && Left_white_line<40)){
		  for(i=0;i<8;i++){
				data_array[i]=ADC_Conversion(i);
			}
		  	Left_white_line = data_array[3];	//Getting data of Left WL Sensor
		  	Center_white_line = data_array[2];  //Getting data of Center WL Sensor
		  	Right_white_line = data_array[1];
		  	print_it();
		  	print_sensor(1,1,1);
		  	move_right();
			_delay_ms(50);
		}
		stop();
	}
	else{
		velocity(200,200);
		while(!(Center_white_line >40 && Left_white_line >40 && Right_white_line >40)){
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);
			print_it();
		  	move_left();
			_delay_ms(50);
		}
		move_left();
		_delay_ms(50);
		stop();
		forward();					 //Forward moving form left lane toward right lane
		Center_white_line=200;
		while(Center_white_line>40)	 //Forward Till White Line of Right Lane not detecsdted
		{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);
			Front_Sharp_Sensor=ADC_Conversion(11);
		  	print_it();
			forward();
			_delay_ms(50);
		}
		forward();	// Correction for white line detection
		_delay_ms(500); 
		
		print_it();
		stop();
		while(!(Center_white_line <40 && Left_white_line<40)){
		  	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);
		  	move_right();
			print_it();
			buzzer_on();
			_delay_ms(50);
		}
		stop();
		buzzer_off();
		Right_Sharp_Sensor=ADC_Conversion(13);
		/*while(time<2*time_required)  //time required to overtake (time in units)
		{
		  time++;
		  line_follower(90,150);
		}*/
		while(Right_Sharp_Sensor>80){
			print_sensor(1,1,9);
			line_follower(140,200);//check from left sensor
			Right_Sharp_Sensor=ADC_Conversion(13);
		}
		time=0;
		/*while(time<2*time_required)  //time required to overtake (time in units)
		{
		  time++;
		  line_follower(90,150);
		}*/
		stop();
		velocity(200,200);
		while(!(Center_white_line >40 && Left_white_line >40 && Right_white_line >40)){
		    Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		  	print_it();
		  	move_right();
	    	_delay_ms(50);
		}
		stop();
		forward();
		Center_white_line=200;
		while(Center_white_line>40)	 //Forward Till White Line of Right Lane not detected
		{
		    Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);
		    Front_Sharp_Sensor=ADC_Conversion(11);
		    print_it();
	        forward();
		}
		forward();	// Correction for white line detection
		_delay_ms(500); 
		stop();
		while(!(Center_white_line <40 && Right_white_line <40)){
		  	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);
		  	print_it();
		  	print_sensor(1,1,1);
		  	move_left();
			_delay_ms(50); 
			buzzer_on();
		}
		buzzer_off();
		stop();
	}
}
//function in which a bot switch to its right lane
void move_right_until_white_line(){
	int b=0;
	while(1){
		Center_white_line=ADC_Conversion(2);//value from center white line sensor
		Left_white_line=ADC_Conversion(3);//value from left white line sensor
		Right_white_line=ADC_Conversion(1);//value from right white line sensor
		//move right until white line is skipped
		if(b==0){
			if(Center_white_line >40 && Left_white_line >40 && Right_white_line >40){move_right();_delay_ms(1000);b=1;stop();}
			move_right();
			buzzer_on();
		}//now move forward until white line is found,this is the track on the right that has been found
		if(b==1){
			if(Center_white_line<40){ forward();_delay_ms(1000);b=2;stop();} 
			forward();
			buzzer_off();
		}
		//move towards left to again get onto track
		if(b==2){
			if(Center_white_line <40 && Right_white_line <40){b=3;stop();}
			move_left();
		}
		//break as soon as the lane is switched
		if(b==3)
			break;
	}
}
//function to move bot using left sharp sensor to find out when to switch the lane again
void forward_until_left_sharp_sensor(){
	Left_Sharp_Sensor=ADC_Conversion(9);
	/*while(time<2*time_required)  //time required to overtake (time in units)
	{
	  time++;
	  line_follower(90,150);
	}*/
	while(Left_Sharp_Sensor>50){
		print_sensor(1,1,9);
		line_follower(140,200);//check from left sensor
		Left_Sharp_Sensor=ADC_Conversion(9);
	}
	/*while(time<2*time_required)  //time required to overtake (time in units)
	{
	  time++;
	  line_follower(90,150);
	}*/
}
//function to move bot using right sharp sensor to find out when to switch the lane again
void forward_until_right_sharp_sensor(){
	Right_Sharp_Sensor=ADC_Conversion(13);
	/*while(time<2*time_required)  //time required to overtake (time in units)
	{
	  time++;
	  line_follower(90,150);
	}*/
	while(Right_Sharp_Sensor>50){
		print_sensor(1,1,13);
		line_follower(140,200);//check from left sensor
		Right_Sharp_Sensor=ADC_Conversion(13);
	}
	/*while(time<2*time_required)  //time required to overtake (time in units)
	{
	  time++;
	  line_follower(90,150);
	}*/
}
//function in which a bot switch to its left lane
void move_left_until_white_line(){
	int b=0;
	while(1){
        Center_white_line=ADC_Conversion(2);//value from center white line sensor
		Left_white_line=ADC_Conversion(3);//value from left white line sensor
		Right_white_line=ADC_Conversion(1);//value from right white line sensor
		//move left until white line is skipped
		if(b==0){
			if(Center_white_line >40 && Left_white_line >40 && Right_white_line >40){move_left();_delay_ms(1000);b=1;stop();}
			move_left();
			buzzer_on();
		}
		//move forward until a new white line is observed, this is white line on the left
		if(b==1){
			if(Center_white_line<40){ forward();_delay_ms(1000);b=2;stop();} 
			forward();
			buzzer_off();
		}
		//move right to get onto track again
		if(b==2){
			if(Center_white_line <40 && Left_white_line <40){move_right();_delay_ms(1000);b=3;stop();}
			print_it();
			move_right();
		}
		//stop as soon as the lanes are switched
		if(b==3)
			break;
	}
}
//intialize all the ports of lcd, zigbee et cetera
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	uart0_init();
	timer5_init();
	sei();   //Enables the global interrupts
}

//Main Function
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	velocity(200,200);
	//forward();
	while(1)
		
	{	
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);  //Getting data of Right WL Sensor
		Left_Sharp_Sensor=ADC_Conversion(9);   //Getting data of Left Sharp Sensor
		line_follower(140, 200);               //calling the line follower subroutine to move the bot on the path
		Front_Sharp_Sensor=ADC_Conversion(11); //Getting data of Front Sharp Sensor
		print_it();
		if(Front_Sharp_Sensor >= 50) // obstacle is near the robot, increase the velocity and move on aprropriate lane and on the buzzer
		{ // Display Working Task on LCD
			//overtaking(); // Overting Subroutine Call
			//turn=0;
			if(turn==1){
				move_right_until_white_line();
				forward_until_left_sharp_sensor();
				move_left_until_white_line();
			}
			else{
				move_left_until_white_line();
				forward_until_right_sharp_sensor();
				move_right_until_white_line();
			}
		}
		else
		{
			//forward();
		}
	}
}
