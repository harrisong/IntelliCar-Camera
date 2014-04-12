/*
 * balance_encoders.h
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <mini_common.h>
#include <MK60_FTM.h>
#include <libutil/clock.h>
#include <assert.h>

#ifndef BALANCE_ENCODERS_H_
#define BALANCE_ENCODERS_H_



class BalanceEncoder{
	private:
		FTMn_e ftmn;
		int16 direction,current, prev;
		uint32 total;
	public:
		BalanceEncoder(int);
		void refresh();
		int16 GetCurrent();
		int32 GetTotal();
		int16 GetDirection();

		void Reset();
};






#endif /* BALANCE_ENCODERS_H_ */
