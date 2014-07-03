/*
 * balance_volt.h
 *
 *  Created on: Jul 3, 2014
 *      Author: harrison
 */

#ifndef BALANCE_VOLT_H_
#define BALANCE_VOLT_H_
#include <mini_common.h>
#include <MK60_adc.h>

namespace camera{
	class BalanceVolt{
	public:
		BalanceVolt(ADCn_Ch_e p):
			port(p){
			adc_init(p);
		}
		float GetVolt(){
			float val = (float) adc_once(port, ADC_16bit) * 3.3 / 65535 * 50 /20;
			return val;
		}
	private:
		ADCn_Ch_e port;
	};
}



#endif /* BALANCE_VOLT_H_ */
