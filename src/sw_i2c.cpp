#include "sw_i2c.h"




void sw_i2c_init(void){	  
	gpio_init (SDA, GPO, 1);
	gpio_init (SCL, GPO, 1);
		/*LPC_GPIO0->FIODIR |= _BV(SDA) | _BV(SCL); //init 2 io dir as out
		gpio_init();
	  LPC_PINCON->PINMODE1 |= _BV(5) | _BV(3);  //
	  LPC_PINCON->PINMODE_OD0 |= _BV(SDA) | _BV(SCL); //	
		LPC_GPIO0->FIOSET |= _BV(SDA) | _BV(SCL); //set 2 io*/
}

void set_io_dir(int dir, PTXn_e ptxn){
	GPIO_CFG mode = dir==OUT ? GPO : GPI;
	gpio_ddr(ptxn, mode);
	/*if(dir){
		GPIO_PDDR_REG(GPIOX_BASE(ptxn)) &= ~(1 << PTn(ptxn));
//		LPC_GPIO0->FIODIR |= _BV(pin); //set as out dir
	}
	else{
		GPIO_PDDR_REG(GPIOX_BASE(ptxn)) &= ~(1 << PTn(ptxn));
//		LPC_GPIO0->FIODIR &= ~_BV(pin);	//set as in dir
	}*/
	systick_delay_ns(DELAYT);
}

void set_scl(void){
	gpio_set(SCL, 1);
//	GPIO_PDOR_REG(GPIOX_BASE(SCL))  |= (1 << PTn(SCL));
//	LPC_GPIO0->FIOSET |= _BV(SCL); //set scl as 1
	systick_delay_ns(DELAYT);
}

void clr_scl(void){
	gpio_set(SCL, 0);
//	GPIO_PDOR_REG(GPIOX_BASE(SCL)) &= ~(1 << PTn(SCL));
//	LPC_GPIO0->FIOCLR |= _BV(SCL); //set scl as 0
	systick_delay_ns(DELAYT);
}

void set_sda(void){				  
	gpio_set(SDA, 1);
//	GPIO_PDOR_REG(GPIOX_BASE(SDA))  |= (1 << PTn(SDA));
//	LPC_GPIO0->FIOSET |= _BV(SDA);
	systick_delay_ns(DELAYT);
}

void clr_sda(void){
	gpio_set(SDA, 0);
//	GPIO_PDOR_REG(GPIOX_BASE(SDA)) &= ~(1 << PTn(SDA));
//	LPC_GPIO0->FIOCLR |= _BV(SDA);
	systick_delay_ns(DELAYT);
}


uint8_t read_sda(void){
	set_io_dir(IN, SDA);
	return gpio_get(SDA);
//	return ((GPIO_PDIR_REG(GPIOX_BASE(SDA)) >> PTn(SDA)) & 0x01);
//	return ((LPC_GPIO0->FIOPIN & _BV(SDA)) ? 1 : 0);
}

void sw_stop(void){
	set_scl();
	set_io_dir(OUT, SDA);
	clr_sda();
	set_sda(); 
}

void sw_start(void){		
	set_scl(); 	
	set_sda();
	clr_sda();
}

void sw_send_byte(uint8_t byte){
	int start_tick = get_ticks();
	int i = 7;
	for(i = 7; i >= 0; i--){	
		clr_scl();
		if(byte & _BV(i)){
			set_sda();
		}
		else{
			clr_sda();
		}
		set_scl();
	}
	set_io_dir(IN, SDA);
	clr_scl();
	while(read_sda()){
		if((start_tick > get_ticks() && (60000 - start_tick + get_ticks() >= TIMEOUT) ) || (libutil::Clock::Time() - start_tick >= TIMEOUT)){
			sw_stop();
			printf("SW_I2C_Error\n\r");
//			nrf905_write("SW_I2C_Error\n\r");
			break;
		}
	}
	set_scl(); 	 
	set_io_dir(OUT, SDA);
	clr_scl();	
	set_sda();
}

uint8_t sw_receive_byte(int nack){
	
	int start_tick = get_ticks();
	uint8_t value = 0;
	int i = 7;		  	
	set_io_dir(IN, SDA);
	for(i = 7; i >= 0; i--){		
		set_scl();
		value |= (read_sda() << i);	
		clr_scl();
	}
	set_io_dir(OUT, SDA);
	if(nack){ 
		set_sda();
	}
	else{	
		clr_sda();
	}						
	return value;
}	  

void sw_i2c_write(uint8_t addr, uint8_t reg, uint8_t data){ 
	sw_start(); 
	sw_send_byte(addr);
	sw_send_byte(reg);	 
	sw_send_byte(data);
	sw_stop();				
}

uint8_t sw_i2c_read(uint8_t addr, uint8_t reg){
	uint8_t value = 0;
	sw_start();
	sw_send_byte(addr);
	sw_send_byte(reg);
	sw_start(); 	
	sw_send_byte(addr | READ);
	value = sw_receive_byte(NACK);
	sw_stop();
	return value;	   
}

void sw_i2c_read_nbytes(uint8_t addr, uint8_t reg, int length, uint8_t * pdata){
	int i = 0;
	for(i = 0; i < length; i++){
		sw_start();		   
		sw_send_byte(addr);
		sw_send_byte(reg + i);
		sw_start(); 	
		sw_send_byte(addr | READ);
		if(i == length - 1){
			*(pdata + i) = sw_receive_byte(NACK);
		}
		else{		
			*(pdata + i) = sw_receive_byte(ACK);
		}		
	}
	sw_stop();   	   
}
