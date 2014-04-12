/*
 * balance_encoder.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <balance_encoder.h>

BalanceEncoder::BalanceEncoder(int n):
	total(0), direction(0), current(0)
{
	libutil::Clock::Init();
	switch(n){
	case 0:
		ftmn = FTM1;
		break;
	case 1:
		ftmn = FTM2;
		break;
	default:
		assert(0);
		break;
	}

	FTM_QUAD_Init(ftmn);
}

void BalanceEncoder::refresh(){
	int32 value = (int32) FTM_QUAD_get(ftmn);

	switch (ftmn){
		case FTM1:
			value = -value;
			break;
		case FTM2:
		default:
			break;
	}

	current = value;
	total += value;
	if(value == 32767) Reset();
	if(current > 0) direction=1;
	else if(current < 0) direction = -1;
	prev = current;
}

int16 BalanceEncoder::GetCurrent(){
	return current;
}

int32 BalanceEncoder::GetTotal(){
	return total;
}

int16 BalanceEncoder::GetDirection(){
	return direction;
}



void BalanceEncoder::Reset(){
	FTM_QUAD_clean(ftmn);
}
