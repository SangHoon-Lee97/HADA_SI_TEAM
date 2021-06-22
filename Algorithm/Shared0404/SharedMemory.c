#include "SharedMemory.h"

void CreateClientXsens(void)
{
	hMampF_CLI_Xsens = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NAME1);
	if (hMampF_CLI_Xsens == NULL)
	{
		printf("1. Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_Xsens = (Xsens_smdat*)MapViewOfFile(hMampF_CLI_Xsens, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Xsens_smdat));

	if (CLI_smdat_Xsens == NULL)
	{
		printf("2. Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_Xsens);
		return 1;
	}
}

void ClosedClientXsens(void)
{
	UnmapViewOfFile(CLI_smdat_Xsens);
	CloseHandle(hMampF_CLI_Xsens);
}

void DataReadXsens(Xsens_smdat* data)
{
	printf("SIM_Count : %d\t SIM_Time : %f\n", data->SIM_count, data->SIM_Time);
	printf("euler_x : %f\t euler_y : %f\t euler_z : %f\n", data->euler_x, data->euler_y, data->euler_z);
	printf("acc_x : %f\t acc_y : %f\t acc_z : %f\n", data->acc_x, data->acc_y, data->acc_z);
	printf("vel_x : %f\t vel_y : %f\t vel_z : %f\n", data->vel_x, data->vel_y, data->vel_z);
	printf("rot_x : %f\t rot_y : %f\t rot_z : %f\n\n", data->rot_x, data->rot_y, data->rot_y);
}

void CreateClientNovA(void)
{
	hMampF_CLI_NovA = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NAME2);
	if (hMampF_CLI_NovA == NULL)
	{
		printf("1. Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_NovA = (NovATel_smdat*)MapViewOfFile(hMampF_CLI_NovA, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(NovATel_smdat));

	if (CLI_smdat_NovA == NULL)
	{
		printf("2. Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_NovA);
		return 1;
	}
}

void ClosedClientNovA(void)
{
	UnmapViewOfFile(CLI_smdat_NovA);
	CloseHandle(hMampF_CLI_NovA);
}

void DataReadNovA(NovATel_smdat* data)
{
	printf("SIM_Count : %d\t SIM_Time : %f\n", data->SIM_count, data->SIM_Time);
	printf("UTChour : %d\t UTCmin : %d\t UTCsec : %d\t UTCmsec : %d\n", data->UTChour, data->UTCmin, data->UTCsec, data->UTCmsec);
	printf("LatDegree : %d\t LatMinute : %f\t NS : %c\n", data->LatDegree, data->LatMinute, data->NS);
	printf("LongDegree : %d\t LongMinute : %f\t EW : %d\n", data->LongDegree, data->LongMinute, data->EW);
	printf("FixQual : %d\t N_Sat : %d\t DOP : %f\n\n", data->FixQual, data->N_Sat, data->DOP);
}

void CreateClientUblox(void)
{
	hMampF_CLI_Ublox = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NAME5);
	if (hMampF_CLI_Ublox == NULL)
	{
		printf("1. Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_Ublox = (Ublox_smdat*)MapViewOfFile(hMampF_CLI_Ublox, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Ublox_smdat));

	if (CLI_smdat_Ublox == NULL)
	{
		printf("2. Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_Ublox);
		return 1;
	}
}

void ClosedClientUblox(void)
{
	UnmapViewOfFile(CLI_smdat_Ublox);
	CloseHandle(hMampF_CLI_Ublox);
}

void DataReadUblox(Ublox_smdat* data)
{
	printf("SIM_Count : %d\t SIM_Time : %f\n", data->SIM_count, data->SIM_Time);
	printf("UTChour : %d\t UTCmin : %d\t UTCsec : %d\t UTCmsec : %d\n", data->UTChour, data->UTCmin, data->UTCsec, data->UTCmsec);
	printf("LatDegree : %d\t LatMinute : %f\t NS : %c\n", data->LatDegree, data->LatMinute, data->NS);
	printf("LongDegree : %d\t LongMinute : %f\t EW : %d\n", data->LongDegree, data->LongMinute, data->EW);
	printf("FixQual : %d\t N_Sat : %d\t DOP : %f\n\n", data->FixQual, data->N_Sat, data->DOP);
}


void CreateClientXsensFlag(void)
{
	hMampF_CLI_Xsens_Flag = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NAME3);

	if (hMampF_CLI_Xsens_Flag == NULL)
	{
		printf("1. Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_Xsens_Flag = (ReadData_flag*)MapViewOfFile(hMampF_CLI_Xsens_Flag, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ReadData_flag));

	if (CLI_smdat_Xsens_Flag == NULL)
	{
		printf("2. Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_Xsens_Flag);
		return 1;
	}
}

void ClosedClientXsensFlag(void)
{
	UnmapViewOfFile(CLI_smdat_Xsens_Flag);
	CloseHandle(hMampF_CLI_Xsens_Flag);
}

void CreateClientNovAFlag(void)
{
	hMampF_CLI_NovA_Flag = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NAME4);

	if (hMampF_CLI_NovA_Flag == NULL)
	{
		printf("1. Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_NovA_Flag = (ReadData_flag*)MapViewOfFile(hMampF_CLI_NovA_Flag, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ReadData_flag));

	if (CLI_smdat_NovA_Flag == NULL)
	{
		printf("2. Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_NovA_Flag);

		return 1;
	}
}

void ClosedClientNovAFlag(void)
{
	UnmapViewOfFile(hMampF_CLI_NovA_Flag);
	CloseHandle(CLI_smdat_NovA_Flag);

}

void CreateClientUbloxFlag(void)
{
	hMampF_CLI_Ublox_Flag = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, CLIE_NAME6);

	if (hMampF_CLI_Ublox_Flag == NULL)
	{
		printf("1. Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	CLI_smdat_Ublox_Flag = (ReadData_flag*)MapViewOfFile(hMampF_CLI_Ublox_Flag, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ReadData_flag));

	if (CLI_smdat_Ublox_Flag == NULL)
	{
		printf("2. Could not map view of file (%d).\n", GetLastError());
		CloseHandle(hMampF_CLI_Ublox_Flag);
		return 1;
	}
}

void ClosedClientUbloxFlag(void)
{
	UnmapViewOfFile(hMampF_CLI_Ublox_Flag);
	CloseHandle(CLI_smdat_Ublox_Flag);
}

void FlagWrite(ReadData_flag data, ReadData_flag* flag)
{
	if (flag == NULL) return -1;
	*flag = data;

	printf("Flag Write : %d\n", data.flag);
}

void CreateServerNavi(void)
{
	hMampF_SER_Navi = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
		0, sizeof(Navigation_smdat), SERV_NAME6);

	if (hMampF_SER_Navi == NULL)
	{
		printf("Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	SERV_smdat_Navi = (Navigation_smdat*)MapViewOfFile(hMampF_SER_Navi, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Navigation_smdat));
}

void ClosedServerNavi(void)
{
	if (SERV_smdat_Navi == NULL) return -1;
	UnmapViewOfFile(SERV_smdat_Navi);
	CloseHandle(hMampF_SER_Navi);
}

void  DataWrite(Navigation_smdat data)
{
	if (SERV_smdat_Navi == NULL) return -1;
	*SERV_smdat_Navi = data;
}

void CreateServerNaviFlag(void)
{
	hMampF_SER_Flag = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
		0, sizeof(ReadData_flag), SERV_NAME6);

	if (hMampF_SER_Flag == NULL)
	{
		printf("Could not open file mapping object (%d).\n", GetLastError());
		return 1;
	}

	SERV_smdat_Flag = (ReadData_flag*)MapViewOfFile(hMampF_SER_Flag, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ReadData_flag));
}

void ClosedServerNaviFlag(void)
{
	if (SERV_smdat_Flag == NULL) return -1;
	UnmapViewOfFile(SERV_smdat_Flag);
	CloseHandle(hMampF_SER_Flag);
}

int  FlagRead(ReadData_flag* data)
{
	printf("\nMain COM flag : %d\n", data->flag);
	return data->flag;
}