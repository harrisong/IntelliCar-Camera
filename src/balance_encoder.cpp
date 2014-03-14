/*
 * balance_encoder.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <balance_encoder.h>

balance_encoder::balance_encoder(FTMn_e n){
	ftmn = n;
	total=0,direction=0,current=0;
	FTM_QUAD_Init(ftmn);
}

void balance_encoder::refresh(){
	current = FTM_QUAD_get(ftmn);
	total += FTM_QUAD_get(ftmn);
	reset();
	if(current > 0) direction=1;
	else if(current < 0) direction = -1;
}

int16 balance_encoder::getcurrent(){
	return current;
}

int16 balance_encoder::gettotal(){
	return total;
}

int16 balance_encoder::getdirection(){
	return direction;
}

void balance_encoder::reset(){
	FTM_QUAD_clean(ftmn);
}
