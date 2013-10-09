//*************************************************************
// Mega 1284 version
// IR test code
// Original idea from 
// http://tthheessiiss.wordpress.com/2009/08/05/dirt-cheap-wireless/
// Bruce Land -- Cornell Univ -- May 2012
//
// Connect D7 thru resistor to (+IR_LED-) to TXD0:: D7--/\/\--|>|----TXD0
// Connect TOSP34156 output to RXD0
// Refer to http://www.vishay.com/docs/81737/tsop341.pdf
// Connect D3 to RS232-transmit to computer
//
// Packet start char is '#'
// Packet end char is '%'
// Do not use # or % in a message
// rx_status = 0 means good data
// rx_status = 2 ; // buffer overrun
// rx_status = 1 ; // no data --> timeout
// rx_status = 3 ; // bad transmitter id
// rx_status = 4 ; // bad check sum

//
// Error rate:
// 2 meters, 100 ohm transmit resistor, bright office environment, no direct sun
// 4800 baud 0/5000 packets -- 1/15000
// 5000 baud 0/30000 -- 1/15000
// 5500 baud 3/12000
// 6000 baud 5/2800
// 7000 baud 5/790
// 8000 baud 5/280
// 9000 baud 5/40
// 9200 baud 49/200
// 2 meters, 330 ohm transmit resistor, bright office environment, no direct sun
// 5000 18/900 better aim 1/1000
// 4800 9/560 better aim 0/1000, 2/500

//*************************************************************              
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>

/* CPU frequency */
#define F_CPU 16000000UL
/* UART baud rate */
// for IR link
#define IR_UART_BAUD  4800
#define PC_UART_BAUD  9600

//time for task -- 1 count is 1 mSec
#define t1 300  //  mSec 
#define t2 201  //  mSec 

#define begin {
#define end }

// ISR
#define SUSPEND cli();
#define RESUME sei();
 
//the task subroutine
void send_task(void);  
void recv_task(void);  	// specify transmitter
void recv_any_task(void);  	// any transmitter
int task_count, error_count ;

void initialize(void); //all the usual mcu stuff 

//timeout counter  
// time to give up and assume you will never get a response   
#define ir_rx_timeout 100 //milliseconds
#define ir_tx_timeout 100
   
// task timers and global time counters (mSec)
volatile unsigned int time1, time2; 
volatile unsigned long tx_send_time, rx_rec_time, time;

//************************
// string buffers and ISR comm buffers
#define buffer_size 64
volatile char ir_tx_buffer[buffer_size], ir_rx_buffer[buffer_size] ;
// count for filling(sending) buffers
volatile char ir_tx_count, ir_rx_count ;
// read flags
volatile char ir_tx_ready, ir_rx_ready ;
// start and end characters for packet
#define start_token '#'
#define end_token '%'

//************************
// bit twiddling macros
#define READ(U, N) (((U) >> (N)) & 1u)
#define SET(U, N) ((void)((U) |= 1u << (N)))
#define CLR(U, N) ((void)((U) &= ~(1u << (N))))
#define FLIP(U, N) ((void)((U) ^= 1u << (N)))

//**********************************************************
//timer 0 overflow ISR --
// increments task timers
// Full duplex transmit/receive
// sends ir character (if ready)
// receives ir char (if valid)
// builds ir receive buffer
ISR (TIMER0_COMPA_vect) 
begin  
	unsigned char c ;
	    
  	//Decrement the time if not already zero
  	if (time1>0) --time1;
	if (time2>0) --time2;
  	time++ ; // running cpu time
	
	//**********************
  	// send an ir char if tx is ready and still char in buffer to send
	// and USART is ready
	if (ir_tx_ready ){ //&& ir_tx_buffer[ir_tx_count]>0
		if (UCSR0A & (1<<UDRE0)) UDR0 = ir_tx_buffer[ir_tx_count++];
		if (ir_tx_buffer[ir_tx_count]==0x00) ir_tx_ready = 0 ; // end of buffer
		if (ir_tx_count >= buffer_size) ir_tx_ready = 0; // buffer overrun
	}
	
	//**********************
  	// recv an ir char if data ready 
  	// otherwise set c to null 
	if (UCSR0A & (1<<RXC0) ) {
		c = UDR0 ; // valid char 
	}
	else c = 0 ; // nonvalid

	//**********************
	// append character to the received string
	// if character is valid and we expect a string
	if (c>0) { //&& (ir_rx_ready==0)) {

		if (c == start_token) { // restart the string
			ir_rx_count = 0 ;
		}

		else if (c == end_token){ // end the string
			ir_rx_buffer[ir_rx_count] = 0x00 ;
			ir_rx_ready = 1 ;
		}

		else { // add to string and check for buffer overrun
			ir_rx_buffer[ir_rx_count++] = c ;
			if (ir_rx_count >= buffer_size) { // buffer overrun
				ir_rx_ready = 2;
				ir_rx_buffer[buffer_size-1] = 0x00 ;
				ir_rx_count = buffer_size -1 ; //???
			}
		}

	} // end if c>0
	
end  

//*********************************************************
// UART 1 interface to use with PC
// --- putchar ---
// uses uart 1
void putchar_uart1(char c)
{
  //if (c == '\n')
  //  uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR1A, UDRE1);
  UDR1 = c;
}

// -- getchar ---
// users uart 1 -- very basic input
char getchar_uart1(void)
{
  uint8_t c;
  loop_until_bit_is_set(UCSR1A, RXC1);
  return c = UDR1;
}

FILE uart1 = FDEV_SETUP_STREAM(putchar_uart1, getchar_uart1, _FDEV_SETUP_RW);

//**********************************************************
// IR send
// Input transmitter id and string packet payload
void ir_send_packet(char tx_id, char ir_data[])
begin
	char ir_tx_ck_sum, ir_tx_ck_sum1, ir_tx_ck_sum2 ;
	char i ;
	
	// compute check sum on input data string only
	ir_tx_ck_sum = 0;
	for (i=0; i<strlen(ir_data); i++)
		ir_tx_ck_sum ^= ir_data[i] ;
	// chop the check sum into two 1/2 bytes and add a bit
	ir_tx_ck_sum1 = 0x10 + (ir_tx_ck_sum & 0x0f) ;
	ir_tx_ck_sum2 = 0x10 + ((ir_tx_ck_sum>>4) & 0x0f) ;

	// format send string
  	sprintf(ir_tx_buffer,"%c%c%s%c%c%c", 
		start_token, tx_id, ir_data, ir_tx_ck_sum1, ir_tx_ck_sum2, end_token );
  	ir_tx_count = 0 ;
  	ir_tx_ready = 1 ;

	tx_send_time = time ;
	// wait 
	while (ir_tx_ready && (time < tx_send_time + ir_tx_timeout)) {};
	
end

//**********************************************************
// IR Receive
// input expected transmitter id
// returns zero if payload is valid and returns payload
// 1 means no data; 2 means buffer overrun; 3 means bad tx id; 4 means bad checksum
char ir_rec_packet(char tx_id, char ir_data[])
begin
	char rx_status = 0;
	char ir_rx_ck_sum, ir_rx_ck_sum_ref;
	char i ;
	char buf_len ;

	buf_len = (char)strlen(ir_rx_buffer) ;
	if (buf_len >= buffer_size) buf_len = buffer_size-1;

	// error check
	if (ir_rx_ready == 2) {rx_status = 2; return rx_status;} // buffer overrun
	if (ir_rx_ready == 0) {rx_status = 1; return rx_status;} // no data -- timeout
	if (ir_rx_buffer[0] != tx_id) {rx_status = 3; return rx_status;} // bad transmitter id
	if (ir_rx_ready != 1) {rx_status = 1; return rx_status;} // invalid data

	// compute receive checksum
	ir_rx_ck_sum = 0 ;
	for (i=1; i<buf_len-2; i++)
		ir_rx_ck_sum ^= ir_rx_buffer[i] ;
	
	ir_rx_ck_sum_ref = (ir_rx_buffer[buf_len-2] & 0x0f) +
		 (ir_rx_buffer[buf_len-1]<<4) ;

    if (ir_rx_ck_sum_ref != ir_rx_ck_sum) 
		{rx_status = 4; return rx_status;}  // bad check sum

	// set up the valid data return stuff
	ir_rx_buffer[0] = ' '; // strip the transmit id
	ir_rx_buffer[buf_len-2] = 0x00; // strip the check sum and trailer
	ir_rx_ready = 0 ;
	//trim initial space using ir_rx_buffer+1
	strlcpy(ir_data, ir_rx_buffer+1, buffer_size) ; 

	return rx_status ;
end


//**********************************************************
// IR Receive ANY transmitter
// 
// returns tx_id (>0x30) if payload is valid and returns payload
// return==1 means no data; 2 means buffer overrun; 4 means bad checksum
char ir_rec_any_packet(char ir_data[])
begin
	char rx_status = 0;
	char ir_rx_ck_sum, ir_rx_ck_sum_ref;
	char i ;
	char buf_len ;

	buf_len = (char)strlen(ir_rx_buffer) ;
	if (buf_len >= buffer_size) buf_len = buffer_size-1;

	// error check
	if (ir_rx_ready == 2) return 2 ; // buffer overrun
	if (ir_rx_ready == 0) return 1 ; // no data -- timeout
	if (ir_rx_ready != 1) {rx_status = 1; return rx_status;} // invalid data
	// if (ir_rx_buffer[0] != tx_id) return 3 ; // bad transmitter id

	// compute receive checksum
	ir_rx_ck_sum = 0 ;
	for (i=1; i<buf_len-2; i++)
		ir_rx_ck_sum ^= ir_rx_buffer[i] ;
	
	ir_rx_ck_sum_ref = (ir_rx_buffer[buf_len-2] & 0x0f) +
		 (ir_rx_buffer[buf_len-1]<<4) ;

	if (ir_rx_ck_sum_ref != ir_rx_ck_sum) return 4 ; // bad check sum

	// set up the valid return stuff
	// if no errors, then return transmitter ID
	if (rx_status == 0) rx_status = ir_rx_buffer[0] ;
	ir_rx_buffer[0] = ' '; // strip the transmit id from string
	ir_rx_buffer[buf_len-2] = 0x00; // strip the check sum and trailer
	ir_rx_ready = 0 ;
	//trim initial space using ir_rx_buffer+1
	strlcpy(ir_data, ir_rx_buffer+1, buffer_size) ;

	return rx_status ;
end
  
//**********************************************************          
// send something
void send_task(void) 
begin  
	char tx_id = 'T'; // note: this is a quoted char
	char ir_tx_data[buffer_size] ;

	// transmit
	sprintf(ir_tx_data,"%ld, %d, %d", time, task_count, error_count); //
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end  	

//**********************************************************          
// receive it back
void recv_task(void) 
begin  
	char rx_id = 'T';  // note: this is a quoted char
	char rx_status = 0 ;
	char ir_rx_data[buffer_size]  ;
	
	// receive
	rx_status = ir_rec_packet(rx_id, ir_rx_data) ;
	if (!rx_status) {
		//ir_rx_data[6] = 0;
		fprintf(&uart1, "PAYLOAD=%s\n\r", ir_rx_data);
	}
	else {
		fprintf(&uart1, "%d%s\n\r", rx_status, ir_rx_data);
		error_count++ ;
	}
	task_count++;
end  	

//**********************************************************          
// receive it back
void recv_any_task(void) 
begin  
//	char rx_id = '1';  // note: this is a quoted char
	char rx_status = 0 ;
	char ir_rx_data[buffer_size]  ;
	
	// receive
	rx_status = ir_rec_any_packet(ir_rx_data) ;
	if (rx_status>=0x30) {
		//ir_rx_data[6] = 0;
		fprintf(&uart1, "TX_ID=%c\n\r", rx_status); // transmitter number
		fprintf(&uart1, "PAYLOAD=%s\n\r", ir_rx_data);// payload
	}
	else {
		fprintf(&uart1, "%d%s\n\r", rx_status, ir_rx_data);
		error_count++ ;
	}
	task_count++;
end  	
//********************************************************** 
//Set it all up
void initialize(void)
begin
  //********************    
  //set up timer 0 for 1 mSec timebase 
  TIMSK0= (1<<OCIE0A);	//turn on timer 0 cmp match ISR 
  OCR0A = 249 ;//
  //set prescalar to divide by 64 (4 microsec/tick)
  TCCR0B= 3; //;	
  // turn on clear-on-match
  TCCR0A= (1<<WGM01) ;
  
  //******************** 

  // IR carrier generator
  // set up timer 2 for square wave with NO ISR
  // 56000 Hz => 1/2 cycle 8.928 microsec = 143 cycles
  // Connect D7 thru resistor to (+)IR_LED(-) to TXD:: D7--/\/\--|>|----TXD
  //
  OCR2A = 142 ; //143 cycles/half-period
  //  count at full rate
  TCCR2B =  1;	
  // set to toggle OC2A, clear on match,
  TCCR2A = (1<<COM2A0) | (1<<WGM21) ;
  // PORT D.7 is OC2A
  DDRD = (1<<PIND7) ;	
  //********************
  
  //init the task timers
  time1=t1;
  time2=t2;
  
  //********************  

  //init UART0 for IR comm
  UBRR0L = (F_CPU / (16UL * IR_UART_BAUD)) - 1;
  UCSR0B = _BV(TXEN0) | _BV(RXEN0); /* tx/rx enable */
  UCSR0C = (1<<UCSZ01) | (1<<USBS0) ; // 7 bit | 2 stop bits

  //init UART1 for PC comm
  UBRR1L = (F_CPU / (16UL * PC_UART_BAUD)) - 1;
  UCSR1B = _BV(TXEN1) ; //| _BV(RXEN1); /* tx/rx enable */
  fprintf(&uart1,"\n\r...Starting IR comm ...\n\r");
  
  //********************
  //crank up the ISRs
  sei();

end  
//==================================================

   
