#pragma once

#ifndef TIME_CHECK
#define	TIME_CHECK

#include	<winsock2.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <math.h>
#include    <string.h>
#include    <memory.h>
#include    <conio.h>
#include    <time.h>
#include    <Windows.h>
#include    <tchar.h>
#include	<sys/types.h>

#define		FINAL_TIME			(double)( 100  )
#define     SAMPLING_TIME		(double)( 0.1 )

extern int   		SIM_count;
extern double		SIM_Time;

extern double		iniTime;
extern double		preTime;;
extern double		curTime;
extern double		delTime;

extern double		IdelTime;

void	TimeInitialization(void);
void	Idle_Time(void);
double	TimeCheck(void);
int		CheckStop(void);
double  CheckWindowsTime(void);

#endif