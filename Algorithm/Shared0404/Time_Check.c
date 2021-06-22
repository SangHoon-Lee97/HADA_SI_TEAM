#include "Time_Check.h"

int   		SIM_count;
double		SIM_Time;

double		iniTime;
double		preTime;
double		curTime;
double		delTime;
double		IdelTime;

void TimeInitialization(void)
{
	iniTime = CheckWindowsTime();
	preTime = iniTime;
}

void Idle_Time(void)
{
	curTime = CheckWindowsTime();
	IdelTime = (curTime - iniTime) - SIM_Time;

	while (1)
	{
		curTime = CheckWindowsTime();
		delTime = curTime - iniTime - SIM_Time;
		if (delTime >= SAMPLING_TIME) break;
	}

	SIM_Time = SAMPLING_TIME * ((double)(SIM_count)+1.0);

	SIM_count++;

}

double TimeCheck(void)
{
	return SIM_Time;
}

int CheckStop(void)
{
	if (SIM_Time < FINAL_TIME) return 0;
	else					   return 1;
}

double CheckWindowsTime(void)
{
	LARGE_INTEGER   liCount, liFreq;

	QueryPerformanceCounter(&liCount);
	QueryPerformanceFrequency(&liFreq);

	return((liCount.QuadPart / ((double)(liFreq.QuadPart))));
};
