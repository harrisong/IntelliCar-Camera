/*
 * balance_encoders.h
 *
 *  Created on: Mar 14, 2014
 *      Author: harrison
 */
#include <mini_common.h>
#include <MK60_FTM.h>
#ifndef BALANCE_ENCODERS_H_
#define BALANCE_ENCODERS_H_



class balance_encoder{
	private:
		FTMn_e ftmn;
		int16 total,direction,current;
	public:


		balance_encoder(FTMn_e n);

		void refresh();

		int16 getcurrent();

		int16 gettotal();

		int16 getdirection();

		void reset();
};






#endif /* BALANCE_ENCODERS_H_ */
