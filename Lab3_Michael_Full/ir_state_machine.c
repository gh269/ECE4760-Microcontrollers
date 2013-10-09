char key_tx_id = 'K'
char lock_rx_id = 'L'

void IR_lock(void* args)  {
	state = IDLE;
	while(1) {
		if (lock_on) {
			if (state == IDLE) {
				char ir_rx_data[buffer_size]; // to store received payload
				trtWait(SEM_RECV_IR);
				rx_status = lock_recv_task(key_tx_id, ir_rx_data); //receive a packet from the key. ir_rx_data should be modified by this.
				trtSignal(SEM_RECV_IR);
				if (rx_status && (ir_rx_data[0] == "r")) {
					state = CHALLENGE;
				}
			}
			else if (state == CHALLENGE) { //base-lock sends challenge to key
				trtWait(SEM_SEND_IR);
				send_lock_challenge(lock_rx_id); // ok to trtWait before sprintf?
				trtSignal(SEM_SEND_IR);
				state = RESPONSE;
			}
			else if (state == RESPONSE) {
				char ir_rx_data[buffer_size];
				uint32_t challenge_time = 0;
				char key_id = 0;
				char rx_status = 0;
				uint32_t time = 0;
				
				trtWait(SEM_RECV_IR);
				rx_status = lock_recv_task(key_tx_id, ir_rx_data); // to store received payload
				trtSignal(SEM_RECV_IR);
				// parse ir_rx_data and checks the key ID ("<" or ">") and time
				fscanf(stdin, "%c %ld", key_id, challenge_time, &ir_rx_data);
				if (!rx_status) {
					if (key_id == AUTHORIZED && (trtCurrentTime - challenge_time > TICKSPERSECOND)) {
						PORTD |= 0x04; //unlock
					}
				}
			else { state == IDLE; }
			}
		}
	}
}

// Returns rx_status and stores payload in ir_rx_data (passed in) if valid
char lock_recv_task( char rx_id, char ir_rx_data[]) 
begin
	//char rx_id = 'K';  // Key identifier K sends "r" to base-lock
	char rx_status = 0 ;
	
	// receive
	rx_status = ir_rec_packet(rx_id, ir_rx_data) ;
	if (!rx_status) {
		//ir_rx_data[6] = 0;
		fprintf(&uart0, "PAYLOAD=%s\n\r", ir_rx_data);
	}
	else {
		fprintf(&uart0, "%d%s\n\r", rx_status, ir_rx_data);
	}
	return rx_status;
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

void send_task_key( char tx_id ) 
begin  
	char tx_id = 'K'; // note: this is a quoted char
	char ir_tx_data[buffer_size] ;

	// transmit
	sprintf(ir_tx_data, "r");
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end 

void send_lock_challenge( char tx_id ) 
begin  
	char ir_tx_data[buffer_size] ;
	sprintf(ir_tx_data, "%ld", trtCurrentTime());
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end 


void send_challenge( void ) 
begin  
	char tx_id = 'L'; // note: this is a quoted char
	char ir_tx_data[buffer_size] ;

	// transmit valid
	if (lock_command) {
		sprintf(ir_tx_data, "%c>%s", lock_command, challange_time_string);
	}
	
	// send payload from tx_id
	ir_send_packet(tx_id, ir_tx_data);
end 