#ifndef SHAREDMEMORY
#define	SHAREDMEMORY

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

#define CLIE_NAME1    TEXT("Xsens_smdat_ReadData")
#define CLIE_NAME2    TEXT("NovATel_smdat_ReadData")
#define CLIE_NAME3    TEXT("Xsens_ReadData_Flag")
#define CLIE_NAME4    TEXT("NovATel_ReadData_Flag")
#define CLIE_NAME5    TEXT("Ublox_smdat_ReadData")
#define CLIE_NAME6    TEXT("Ubox_ReadData_Flag")
#define SERV_NAME5    TEXT("Navigation_smdat_WriteData")
#define SERV_NAME6    TEXT("Navigation_Conduct_Flag")

#define MAX_LIDAR_NUM2		641			//500360

typedef struct 
{
	int		SIM_count;
	double  SIM_Time;
	double  euler_x;
	double  euler_y;
	double  euler_z;
	double  acc_x;
	double  acc_y;
	double  acc_z;
	double  vel_x;
	double  vel_y;
	double  vel_z;
	double  rot_x;
	double  rot_y;
	double  rot_z;
}Xsens_smdat;

typedef struct 
{
	int		SIM_count;
	double  SIM_Time;
	long	UTChour;
	long	UTCmin;
	long	UTCsec;
	long	UTCmsec;
	int		LatDegree;
	double	LatMinute;
	char	NS;
	int		LongDegree;
	double	LongMinute;
	char	EW;
	char	FixQual;
	int		N_Sat;
	double   DOP;
	int		flag;
}NovATel_smdat;

typedef struct
{
	int		SIM_count;
	double  SIM_Time;
	long	UTChour;
	long	UTCmin;
	long	UTCsec;
	long	UTCmsec;
	int		LatDegree;
	double	LatMinute;
	char	NS;
	int		LongDegree;
	double	LongMinute;
	char	EW;
	char	FixQual;
	int		N_Sat;
	double   DOP;
	int		flag;
}Ublox_smdat;

typedef struct
{
	int		SIM_count;
	double  SIM_Time;
	double  Roll;
	double  Pitch;
	double  Yaw;
	double  Longtitude;
	double  Latitude;
	double  East_Vel;
	double  North_Vel;
	double  dis[MAX_LIDAR_NUM2];
}Navigation_smdat;

typedef struct Flag
{
	int		flag;
}ReadData_flag;

HANDLE		 hMampF_CLI_Xsens; // handle, Mampping File, Server
Xsens_smdat* CLI_smdat_Xsens;
Xsens_smdat  dat_buf_Xsens;

HANDLE         hMampF_CLI_NovA; // handle, Mampping File, Server
NovATel_smdat* CLI_smdat_NovA;
NovATel_smdat  dat_buf_NovA;

HANDLE         hMampF_CLI_Ublox; // handle, Mampping File, Server
NovATel_smdat* CLI_smdat_Ublox;
NovATel_smdat  dat_buf_Ublox;

HANDLE		   hMampF_CLI_Xsens_Flag; // handle, Mampping File, Server
ReadData_flag* CLI_smdat_Xsens_Flag;
ReadData_flag  dat_buf_Xsens_Flag;

HANDLE		   hMampF_CLI_NovA_Flag; // handle, Mampping File, Server
ReadData_flag* CLI_smdat_NovA_Flag;
ReadData_flag  dat_buf_NovA_Flag;


HANDLE		   hMampF_CLI_Ublox_Flag; // handle, Mampping File, Server
ReadData_flag* CLI_smdat_Ublox_Flag;
ReadData_flag  dat_buf_Ublox_Flag;

HANDLE			  hMampF_SER_Navi; // handle, Mampping File, Server
Navigation_smdat* SERV_smdat_Navi;
Navigation_smdat  dat_buf_Navi;

HANDLE		   hMampF_SER_Flag; // handle, Mampping File, Server
ReadData_flag* SERV_smdat_Flag;
ReadData_flag  dat_buf_Flag;

void CreateClientXsens(void);
void ClosedClientXsens(void);
void DataReadXsens(Xsens_smdat* data);

void CreateClientNovA(void);
void ClosedClientNovA(void);
void DataReadNovA(NovATel_smdat* data);

void CreateClientUblox(void);
void ClosedClientUblox(void);
void DataReadUblox(Ublox_smdat* data);

void CreateClientXsensFlag(void);
void ClosedClientXsensFlag(void);

void CreateClientNovAFlag(void);
void ClosedClientNovAFlag(void);

void CreateClientUbloxFlag(void);
void ClosedClientUbloxFlag(void);

void FlagWrite(ReadData_flag data, ReadData_flag* flag);

void CreateServerNavi(void);
void ClosedServerNavi(void);
void DataWrite(Navigation_smdat data);

void CreateServerNaviFlag(void);
void ClosedServerNaviFlag(void);
int  FlagRead(ReadData_flag* data);

#endif