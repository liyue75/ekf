#include <uxr/client/util/time.h>
//#include <time.h>
#include "xtimer.h"
#include "timex.h"

#ifdef WIN32
#include <Windows.h>
#endif

#ifndef _CURRENT_TIME
#define _CURRENT_TIME 1603942905000000000   //current time 2020/10/29 11:45
#endif

//==================================================================
//                             PUBLIC
//==================================================================
int64_t uxr_millis(void)
{
    return (xtimer_now_usec64() / 1000);
}

int64_t uxr_nanos(void)
{
    //struct timespec ts;
	//    clock_gettime(CLOCK_REALTIME, &ts);
	//    return (((int64_t)ts.tv_sec) * 1000000000) + ts.tv_nsec;
	//uint64_t t = xtimer_now_usec64();
	return (xtimer_now_usec64() * 1000 /*+ _CURRENT_TIME*/);
}
