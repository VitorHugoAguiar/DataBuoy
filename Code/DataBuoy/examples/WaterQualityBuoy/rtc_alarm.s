#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

/* Define variables, which go into .bss section (zero-initialized data) */
.bss

/* Next input signal edge expected: 1 (negative) or 0 (positive) */
.global next_edge
next_edge:
.long 0

/* RTC IO number used to sample the input signal. Set by main program. */
.global io_number
io_number:
.long 0

/* Code goes into .text section */
.text
.global entry
entry:
/* Load io_number */
move r3, io_number
ld r3, r3, 0

/* Lower 16 IOs and higher need to be handled separately,
   because r0-r3 registers are 16 bit wide.
   Check which IO this is.
*/
move r0, r3
jumpr read_io_high, 16, ge

/* Read the value of lower 16 RTC IOs into R0 */
READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
rsh r0, r0, r3
jump read_done

/* Read the value of RTC IOs 16-17, into R0 */
read_io_high:
READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2)
sub r3, r3, 16
rsh r0, r0, r3

read_done:
and r0, r0, 1
/* State of input changed? */
move r3, next_edge
ld r3, r3, 0
add r3, r0, r3
and r3, r3, 1
jump edge_detected, eq
/* Not changed */
jump entry

.global edge_detected
edge_detected:
/* Flip next_edge */
move r3, next_edge
ld r2, r3, 0
add r2, r2, 1
and r2, r2, 1
st r2, r3, 0

/*compare r0 with r2(next_edge value) to see if the GPIO is 1 (negative) or 0 (positive);*/
move r0, r2
jumpr wake_up, 1, eq
jump entry

//value within range, end the program
.global exit
exit:
halt

.global wake_up
wake_up:
//Check if the system can be woken up
READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
and r0, r0, 1
jump exit, eq


//Wake up the SoC, end program
wake
WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
halt
