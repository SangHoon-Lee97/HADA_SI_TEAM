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
//	QueryPerformanceCounter(&liCount); // 시간 함수 밀리 세컨드 단위로 측정이 가능하다
//	QueryPerformanceFrequency(&liFreq); // 진동수/[sec]
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
//	//LPBYTE pMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형
//	////gps
//	//HANDLE dMemoryMap = NULL;
//	//LPBYTE qMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형
//
//
//
//	//imu
//	hMemoryMap = OpenFileMapping(
//		FILE_MAP_READ,	  // read/write access
//		FALSE,	          // do not inherit the name
//		IMU_SM);          // 공유 파일맵의 이름 - Uique 해야한다.
//
//
//	if (!hMemoryMap) {
//		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
//			GetLastError());
//		return FALSE;
//	}
//
//	pMemoryMap = (BYTE*)MapViewOfFile(
//		hMemoryMap,				// 파일맵의 핸들
//		FILE_MAP_READ,    // 액세스 모드 - 현재는 쓰기
//		0,						// 메모리 시작번지부터의 이격된 상위 32비트 
//		0,						// 메모리 시작번지부터의 이격된 하위 32비트
//		0);						// 사용할 메모리 블록의 크기 - 0이면 설정한 전체 메모리
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
//		GPS_SM);          // 공유 파일맵의 이름 - Uique 해야한다.
//
//
//	if (!dMemoryMap) {
//		//::MessageBox(NULL, L"공유 메모리를 생성할 수 없습니다.", L"Error", MB_OK);
//		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
//			GetLastError());
//		return 0;
//	}
//
//	qMemoryMap = (BYTE*)MapViewOfFile(
//		dMemoryMap,				// 파일맵의 핸들
//		FILE_MAP_READ,    // 액세스 모드 - 현재는 쓰기
//		0,						// 메모리 시작번지부터의 이격된 상위 32비트 
//		0,						// 메모리 시작번지부터의 이격된 하위 32비트
//		0);						// 사용할 메모리 블록의 크기 - 0이면 설정한 전체 메모리
//
//	if (!qMemoryMap)
//	{
//		CloseHandle(dMemoryMap);
//		//::MessageBox(NULL, L"공유 메모리를 열수 없습니다.",  L"Error", MB_OK);
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
//		// 여기서 import data해주고 gps_smdat에다가 갚 넣어주면 댐
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