#include <cstdint>
#include <mini_common.h>
#include <hw_common.h>

#include <balance_encoder.h>
#include <MK60_pit.h>
#include <vectors.h>

balance_encoder e1(FTM1);
balance_encoder e2(FTM2);
__ISR void Pit0Handler()
{
	e1.refresh();
	e2.refresh();
	PIT_Flag_Clear(PIT0);
}

int main(void)
{
	SetIsr(PIT0_VECTORn, Pit0Handler);
	pit_init_ms(PIT0, 500);
	EnableIsr(PIT0_VECTORn);




    return 0;
}


