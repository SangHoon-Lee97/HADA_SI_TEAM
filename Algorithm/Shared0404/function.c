//#ifndef function
//#define function
//
//#include "Include.h"
//#include "Time_variable.h"
//#include "variable.h"
//
//// ==========================================
////---------------------------
//// check windows Time
////---------------------------
//double GetWindowTime(void) {
//	LARGE_INTEGER   liCount, liFreq;
//	QueryPerformanceCounter(&liCount); // �ð� �Լ� �и� ������ ������ ������ �����ϴ�
//	QueryPerformanceFrequency(&liFreq); // ������/[sec]
//	return((liCount.QuadPart / ((double)(liFreq.QuadPart))) * 1000.0);
//};
//
//void CheckTime(void)
//{
//	while (1) {
//		curTime = GetWindowTime() * 0.001;   // [ms]
//		delTime = curTime - iniTime - simTime;
//
//		if (delTime >= Ts) {
//			break;
//		}
//	}
//	simTime = ((double)simcnt + 1.0) * Ts;
//	simcnt = simcnt + 1;
//}
//
//
//
//void MemOpen()//int argc, _TCHAR* argv[])
//{
//	////imu
//	//HANDLE hMemoryMap = NULL;
//	//LPBYTE pMemoryMap = NULL; // LPBYTE�� unsigned char�� ��������
//	////gps
//	//HANDLE dMemoryMap = NULL;
//	//LPBYTE qMemoryMap = NULL; // LPBYTE�� unsigned char�� ��������
//
//
//
//	//imu
//	hMemoryMap = OpenFileMapping(
//		FILE_MAP_READ,	  // read/write access
//		FALSE,	          // do not inherit the name
//		IMU_SM);          // ���� ���ϸ��� �̸� - Uique �ؾ��Ѵ�.
//
//
//	if (!hMemoryMap) {
//		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
//			GetLastError());
//		return FALSE;
//	}
//
//	pMemoryMap = (BYTE*)MapViewOfFile(
//		hMemoryMap,				// ���ϸ��� �ڵ�
//		FILE_MAP_READ,    // �׼��� ��� - ����� ����
//		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ 
//		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
//		0);						// ����� �޸� ����� ũ�� - 0�̸� ������ ��ü �޸�
//
//	if (!pMemoryMap)
//	{
//		CloseHandle(hMemoryMap);
//		printf("COULD NOT OPEN THE SHARED MEMORY\n");
//		return FALSE;
//	}
//	printf("IMU SHARED MEMORY IS CREATED\n");
//
//	//__int64 check = 0;
//
//	//IMU_DATA* 
//	imu_smdat = (IMU_DATA*)pMemoryMap;
//
//	//gps
//	dMemoryMap = OpenFileMapping(
//		FILE_MAP_READ,	  // read/write access
//		FALSE,	          // do not inherit the name
//		GPS_SM);          // ���� ���ϸ��� �̸� - Uique �ؾ��Ѵ�.
//
//
//	if (!dMemoryMap) {
//		//::MessageBox(NULL, L"���� �޸𸮸� ������ �� �����ϴ�.", L"Error", MB_OK);
//		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
//			GetLastError());
//		return 0;
//	}
//
//	qMemoryMap = (BYTE*)MapViewOfFile(
//		dMemoryMap,				// ���ϸ��� �ڵ�
//		FILE_MAP_READ,    // �׼��� ��� - ����� ����
//		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ 
//		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
//		0);						// ����� �޸� ����� ũ�� - 0�̸� ������ ��ü �޸�
//
//	if (!qMemoryMap)
//	{
//		CloseHandle(dMemoryMap);
//		//::MessageBox(NULL, L"���� �޸𸮸� ���� �����ϴ�.",  L"Error", MB_OK);
//		printf("COULD NOT OPEN THE SHARED MEMORY\n");
//		return 0;
//	}
//
//	__int64 check = 0;
//
//	//GPS_DATA* 
//	gps_smdat = (GPS_DATA*)qMemoryMap;
//
//	printf("GPS SHARED MEMORY IS CREATED\n");
//
//}
//
//void MemWrite()
//{
//	//do
//	//{
//		// ���⼭ import data���ְ� gps_smdat���ٰ� �� �־��ָ� ��
//
//	unsigned int index_imu = imu_smdat->index_imu;
//	double       roll = imu_smdat->roll;
//	double       pitch = imu_smdat->pitch;
//	double       yaw = imu_smdat->yaw;
//
//	unsigned int index_gps = gps_smdat->index_gps;
//	double      latitude = gps_smdat->latitude;
//	double      longitude = gps_smdat->longitude;
//
//	//strcpy(buffer, imu_smdat->buffer);
//
//	printf("%d %f %f %f %d %f %f \n", index_imu, roll, pitch, yaw, index_gps, latitude, longitude);
//	//printf("%d %f %f \n", index_gps, latitude, longitude);
//	//printf("%d %f %f\n", index, latitude, longitude);
//
//	/*if(check++%100000 == 0)
//	{
//		if(_kbhit() != 0)
//			break;
//	}*/
//	//while (1) {
//	//	curTime = GetWindowTime() * 0.001;   // [ms]
//	//	delTime = curTime - iniTime - simTime;
//
//	//	if (delTime >= Ts) {
//	//		break;
//	//	}
//	//}
//
////	simTime = ((double)simcnt + 1.0) * Ts;
////	simcnt = simcnt + 1;
////} while (simTime < RUNTIME);
//}
//
//void MemClose() {
//	//if (pMemoryMap)
//	//	UnmapViewOfFile(pMemoryMap);
//
//	//if (hMemoryMap)
//	//	CloseHandle(hMemoryMap);
//
//	//if (qMemoryMap)
//	//	UnmapViewOfFile(qMemoryMap);
//
//	//if (dMemoryMap)
//	//	CloseHandle(dMemoryMap);
//	return 0;
//
//}
//
//#endif