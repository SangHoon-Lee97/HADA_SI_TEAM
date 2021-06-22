#define _CRT_SECURE_NO_WARNINGS
// ShareServer.cpp : Defines the entry point for the console application.
//


/// <summary>
/// 
/// </summary>

typedef struct
{
	unsigned int index_imu;
	double roll;
	double pitch;
	double yaw;
} IMU_DATA;

typedef struct
{
	unsigned int index_gps;
	double latitude;
	double longitude;

} GPS_DATA;

//=====================================================================
//=====================================================================

#define RUNTIME                       (double)             (40.0)      //[sec]
#define FREQ                          (double)             (20.0)      //[Hz]
#define N_data                        (unsigned int)       (RUNTIME*FREQ)
#define CoordSys                                           (XDI_CoordSysNed) 


// =========================================
// =========================================
double iniTime = 0.0;
double curTime = 0.0;
double delTime = 0.0;
double simTime = 0.0;
double task1 = 0.0;
double task2 = 0.0;
double task3 = 0.0;
double Ts = 1 / FREQ;
int    simcnt = 0;

// ===========
TCHAR IMU_SM[] = TEXT("imu data");
TCHAR GPS_SM[] = TEXT("gps data");
// ==========================================
//---------------------------
// check windows Time
//---------------------------
double GetWindowTime(void) {
	LARGE_INTEGER   liCount, liFreq;
	QueryPerformanceCounter(&liCount); // 시간 함수 밀리 세컨드 단위로 측정이 가능하다
	QueryPerformanceFrequency(&liFreq); // 진동수/[sec]
	return((liCount.QuadPart / ((double)(liFreq.QuadPart))) * 1000.0);
};


int _tmain(int argc, _TCHAR* argv[])
{
	//imu
	HANDLE hMemoryMap = NULL;
	LPBYTE pMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형
	//gps
	HANDLE dMemoryMap = NULL;
	LPBYTE qMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형



	//imu
	hMemoryMap = CreateFileMapping(
		INVALID_HANDLE_VALUE,//(HANDLE)0xffffffff, // 파일 맵의 핸들, 초기에 0xffffffff를 설정한다.
		NULL,				// 보안 속성
		PAGE_READWRITE,     // 읽고/쓰기 속성
		0,					// 64비트 어드레스를 사용한다. 상위 32비트 - 메모리의 크기
		sizeof(IMU_DATA),   // 하위 32비트 - 여기선LPBYTE 타입.
		IMU_SM);    // 공유 파일맵의 이름 - Uique 해야한다.


	if (!hMemoryMap) {
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}

	pMemoryMap = (BYTE*)MapViewOfFile(
		hMemoryMap,				// 파일맵의 핸들
		FILE_MAP_ALL_ACCESS,    // 액세스 모드 - 현재는 쓰기
		0,						// 메모리 시작번지부터의 이격된 상위 32비트 
		0,						// 메모리 시작번지부터의 이격된 하위 32비트
		0);						// 사용할 메모리 블록의 크기 - 0이면 설정한 전체 메모리

	if (!pMemoryMap)
	{
		CloseHandle(hMemoryMap);
		printf("COULD NOT OPEN THE SHARED MEMORY");
		return 1;
	}
	printf("IMU SHARED MEMORY IS CREATED");

	//__int64 check = 0;

	IMU_DATA* imu_smdat = (IMU_DATA*)pMemoryMap;

	//gps
	dMemoryMap = CreateFileMapping(
		INVALID_HANDLE_VALUE,//(HANDLE)0xffffffff, // 파일 맵의 핸들, 초기에 0xffffffff를 설정한다.
		NULL,				// 보안 속성
		PAGE_READWRITE,     // 읽고/쓰기 속성
		0,					// 64비트 어드레스를 사용한다. 상위 32비트 - 메모리의 크기
		sizeof(GPS_DATA),   // 하위 32비트 - 여기선LPBYTE 타입.
		GPS_SM);    // 공유 파일맵의 이름 - Uique 해야한다.


	if (!dMemoryMap) {
		//::MessageBox(NULL, L"공유 메모리를 생성할 수 없습니다.", L"Error", MB_OK);
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}

	qMemoryMap = (BYTE*)MapViewOfFile(
		dMemoryMap,				// 파일맵의 핸들
		FILE_MAP_ALL_ACCESS,    // 액세스 모드 - 현재는 쓰기
		0,						// 메모리 시작번지부터의 이격된 상위 32비트 
		0,						// 메모리 시작번지부터의 이격된 하위 32비트
		0);						// 사용할 메모리 블록의 크기 - 0이면 설정한 전체 메모리

	if (!qMemoryMap)
	{
		CloseHandle(dMemoryMap);
		//::MessageBox(NULL, L"공유 메모리를 열수 없습니다.",  L"Error", MB_OK);
		printf("COULD NOT OPEN THE SHARED MEMORY");
		return 1;
	}

	__int64 check = 0;

	GPS_DATA* gps_smdat = (GPS_DATA*)qMemoryMap;

	iniTime = GetWindowTime() * 0.001;
	do
	{
		// 여기서 import data해주고 gps_smdat에다가 갚 넣어주면 댐

		unsigned int index_imu = imu_smdat->index_imu;
		double       roll = imu_smdat->roll;
		double       pitch = imu_smdat->pitch;
		double       yaw = imu_smdat->yaw;

		unsigned int index_gps = gps_smdat->index_gps;
		double      latitude = gps_smdat->latitude;
		double      longitude = gps_smdat->longitude;

		//strcpy(buffer, imu_smdat->buffer);

		printf("%d %f %f %f %d %f %f \n", index_imu, roll, pitch, roll, index_gps, latitude, longitude);
		//printf("%d %f %f\n", index, latitude, longitude);

		/*if(check++%100000 == 0)
		{
			if(_kbhit() != 0)
				break;
		}*/
		while (1) {
			curTime = GetWindowTime() * 0.001;   // [ms]
			delTime = curTime - iniTime - simTime;

			if (delTime >= Ts) {
				break;
			}
		}

		simTime = ((double)simcnt + 1.0) * Ts;
		simcnt = simcnt + 1;
	} while (simTime < RUNTIME);

	if (pMemoryMap)
		UnmapViewOfFile(pMemoryMap);

	if (hMemoryMap)
		CloseHandle(hMemoryMap);

	if (qMemoryMap)
		UnmapViewOfFile(pMemoryMap);

	if (dMemoryMap)
		CloseHandle(hMemoryMap);
	return 0;

}

