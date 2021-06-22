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
	QueryPerformanceCounter(&liCount); // �ð� �Լ� �и� ������ ������ ������ �����ϴ�
	QueryPerformanceFrequency(&liFreq); // ������/[sec]
	return((liCount.QuadPart / ((double)(liFreq.QuadPart))) * 1000.0);
};


int _tmain(int argc, _TCHAR* argv[])
{
	//imu
	HANDLE hMemoryMap = NULL;
	LPBYTE pMemoryMap = NULL; // LPBYTE�� unsigned char�� ��������
	//gps
	HANDLE dMemoryMap = NULL;
	LPBYTE qMemoryMap = NULL; // LPBYTE�� unsigned char�� ��������



	//imu
	hMemoryMap = CreateFileMapping(
		INVALID_HANDLE_VALUE,//(HANDLE)0xffffffff, // ���� ���� �ڵ�, �ʱ⿡ 0xffffffff�� �����Ѵ�.
		NULL,				// ���� �Ӽ�
		PAGE_READWRITE,     // �а�/���� �Ӽ�
		0,					// 64��Ʈ ��巹���� ����Ѵ�. ���� 32��Ʈ - �޸��� ũ��
		sizeof(IMU_DATA),   // ���� 32��Ʈ - ���⼱LPBYTE Ÿ��.
		IMU_SM);    // ���� ���ϸ��� �̸� - Uique �ؾ��Ѵ�.


	if (!hMemoryMap) {
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}

	pMemoryMap = (BYTE*)MapViewOfFile(
		hMemoryMap,				// ���ϸ��� �ڵ�
		FILE_MAP_ALL_ACCESS,    // �׼��� ��� - ����� ����
		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ 
		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
		0);						// ����� �޸� ����� ũ�� - 0�̸� ������ ��ü �޸�

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
		INVALID_HANDLE_VALUE,//(HANDLE)0xffffffff, // ���� ���� �ڵ�, �ʱ⿡ 0xffffffff�� �����Ѵ�.
		NULL,				// ���� �Ӽ�
		PAGE_READWRITE,     // �а�/���� �Ӽ�
		0,					// 64��Ʈ ��巹���� ����Ѵ�. ���� 32��Ʈ - �޸��� ũ��
		sizeof(GPS_DATA),   // ���� 32��Ʈ - ���⼱LPBYTE Ÿ��.
		GPS_SM);    // ���� ���ϸ��� �̸� - Uique �ؾ��Ѵ�.


	if (!dMemoryMap) {
		//::MessageBox(NULL, L"���� �޸𸮸� ������ �� �����ϴ�.", L"Error", MB_OK);
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
			GetLastError());
		return 1;
	}

	qMemoryMap = (BYTE*)MapViewOfFile(
		dMemoryMap,				// ���ϸ��� �ڵ�
		FILE_MAP_ALL_ACCESS,    // �׼��� ��� - ����� ����
		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ 
		0,						// �޸� ���۹��������� �̰ݵ� ���� 32��Ʈ
		0);						// ����� �޸� ����� ũ�� - 0�̸� ������ ��ü �޸�

	if (!qMemoryMap)
	{
		CloseHandle(dMemoryMap);
		//::MessageBox(NULL, L"���� �޸𸮸� ���� �����ϴ�.",  L"Error", MB_OK);
		printf("COULD NOT OPEN THE SHARED MEMORY");
		return 1;
	}

	__int64 check = 0;

	GPS_DATA* gps_smdat = (GPS_DATA*)qMemoryMap;

	iniTime = GetWindowTime() * 0.001;
	do
	{
		// ���⼭ import data���ְ� gps_smdat���ٰ� �� �־��ָ� ��

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

