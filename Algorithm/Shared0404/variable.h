#ifndef VARIABLE
#define VARIABLE

#define R0      6373668.30
#define LAT0    36.103177
#define LON0    129.385917
#define PI      3.1415926535
#define DEG2RAD PI/180
#define RAD2DEG 180/PI
#define K_GLOB  (double) (5.0)
#define Body_L  (double) (0.5) // [m]
#define MAX_STEER (int)(28)
#define MIN_DIST  (int)(1) //[m]

typedef struct
{
	int     SIM_count;
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
} IMU_DATA;

typedef struct
{
	int      SIM_count;
	double  SIM_Time;
	long   UTChour;
	long   UTCmin;
	long   UTCsec;
	long   UTCmsec;
	int      LatDegree;
	float   LatMinute;
	char   NS;
	int      LongDegree;
	float   LongMinute;
	char   EW;
	char   FixQual;
	int      N_Sat;
	float   DOP;
	int      flag;
} GPS_DATA;

typedef struct
{
	double      Lon;
	double      Lat;
	double      Roll;
	double      Pitch;
	double      Yaw;

} SEND_DATA;

GPS_DATA* gps_smdat;
IMU_DATA* imu_smdat;
SEND_DATA* send_smdat;

unsigned int  index_imu = 0;
double        roll = 0.0;
double        pitch = 0.0;
double        yaw = 0.0;
double        accX = 0.0;
double        accY = 0.0;
double        accZ = 0.0;

unsigned int index_gps = 0;
double       latitude = 3;
double       longitude = 3;

//imu
HANDLE hMemoryMap = NULL;
LPBYTE pMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형
//gps
HANDLE dMemoryMap = NULL;
LPBYTE qMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형
//send data
HANDLE sMemoryMap = NULL;
LPBYTE xMemoryMap = NULL; // LPBYTE는 unsigned char의 포인터형

FILE* pFile;
//=====================================================================
#define RUNTIME                       (double)             (10.0)      //[sec]
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
double Ts = 0.05;
static int    simcnt = 0;
int    test = 0;
static int    idx_way = 0;


// ===========
TCHAR IMU_SM[] = TEXT("Xsens_smdat_ReadData");
TCHAR GPS_SM[] = TEXT("NovATel_smdat_ReadData");
TCHAR SEND_SM[] = TEXT("Filter_smdat_SendData");
// matrix cal
double Ygps[2][1] = { 0, };
double B_acc[4][1] = { 0, };
double A_eX[4][1];
double B_eX[4][1];
double C_eX[2][1];
double Kf_gps[4][1];
double _Ygps[2][1];
double Y_gps[2][1];
static double Vec_BodyToWay[1][2] = { 0.0, 0.0};


// Buffer
static int    buf_Index[N_data] = { 0, };
static double buf_Raw_Lat[N_data] = { 0, };
static double buf_Raw_Lon[N_data] = { 0, };
static double buf_Raw_Roll[N_data] = { 0, };
static double buf_Raw_Pitch[N_data] = { 0, };
static double buf_Raw_Yaw[N_data] = { 0, };
static double buf_Raw_AccX[N_data] = { 0, };
static double buf_Raw_AccY[N_data] = { 0, };
static double buf_Raw_AccZ[N_data] = { 0, };
static double buf_Filtered_Lat[N_data] = { 0, };
static double buf_Filtered_Lon[N_data] = { 0, };
static double buf_Filtered_EVel[N_data] = { 0, };
static double buf_Filtered_NVel[N_data] = { 0, };


// TM Conversion
static double buf_TM_Lat[N_data] = { 0, };
static double buf_TM_Lon[N_data] = { 0, };


// Pursuit Guidance Variable
static double buf_gamma[N_data] = { 0, };
static double buf_Dist_R[N_data] = { 0, };
static double buf_lambda[N_data] = { 0, };
static double buf_delta[N_data] = { 0, };
static double buf_delta_f[N_data] = { 0, };
static double GPS_waypoint_Lat[N_data] = { 36.1022720713979,36.1021844686696,36.1021128629237,36.1020954779017 ,36.1022231827292,36.1023179137859,36.1023517716483,36.1024909444706};
static double GPS_waypoint_Lon[N_data] = { 129.384665300645,129.384748096996,129.384827691525,129.384855924330,129.385023271002,129.385166410700,129.385260348791,129.385569212235 };
static double GPS_waypoint_Lat_now = 0;
static double GPS_waypoint_Lon_now = 0;
static double buf_GPS_waypoint_Lat_now[N_data] = { 0.0, };
static double buf_GPS_waypoint_Lon_now[N_data] = { 0.0, };

double eX[4][1] = {
   {0},{0},{0},{0}
};

double A_bar[4][4] = {
   { 1, 0,      0.05,          0},
   { 0, 1,         0,   0.061947},
   { 0, 0,         1,  -0.135258},
   { 0, 0,  0.135258,          1}
};

double B_bar[4][2] = {
   { 0,0 },
   { 0,0 },
   { 0.05,0 },
   { 0,0.05 }
};

double Kf_inf[4][2] = {
   {  1.756868,  0.00775719},
   {0.00775719,    1.825937},
   {  1.734019,   -0.354838},
   {  0.450310,    2.137799}
};

double C[2][4] = {
   { 1, 0, 0, 0 },
   { 0, 1, 0, 0 }
};

double U_acc[3][1] = { 0, };


// TM Conversion
#define Lat_std  (double)(38.0)   // 동부 기준 투영원점 위도
#define Lon_std  (double)(129.0)  // 동부 기준 투영원점 경도
#define Flat_f   (double)(1 / 298.257222101)  // GRS80 편평률
#define Long_R   (double)(6378137)            // GRS80 타원체의 장반경
#define Short_R  (double)(Long_R*(1-Flat_f))  // GRS80 타원체의 단반경
#define K_scale  (double)(1.0)                // 원점 축척 계수
#define dy       (double)(200000)
#define dx       (double)(600000)
#define Ecc_1    (double)((pow(Long_R,2) - pow(Short_R,2))/pow(Long_R,2))   // 제 1이심률
#define Ecc_2    (double)((pow(Long_R,2) - pow(Short_R,2))/pow(Short_R,2))  // 제 2이심률
#define M0       (double)(Long_R*((1-Ecc_1/4-3*pow(Ecc_1,2)/64-5*pow(Ecc_1,3)/256)*Lat_std*DEG2RAD)-(3*Ecc_1/8+3*pow(Ecc_1,2)/32+45*pow(Ecc_1,3)/1024)*sin(2*Lat_std*DEG2RAD)+(15*pow(Ecc_1,2)/256+45*pow(Ecc_1,3)/1024)*sin(4*Lat_std*DEG2RAD)-35*pow(Ecc_1,3)/3072*sin(6*Lat_std*DEG2RAD))

double Curr_Lat = 0.0;
double Curr_Lon = 0.0;
double TM_Lat = 0.0;
double TM_Lon = 0.0;
double T = 0.0;
double C_ = 0.0;
double A = 0.0;
double N = 0.0;
double M = 0.0;


// Pursuit Guidance
static double delta = 0.0;
static double gamma = 0.0;
static double Dist_R = 0.0;
static double lambda = 0.0;
static double delta_f = 0.0;


//ERP42
//-----------------------------------------------------------------------//
// ERP 42 PROTOCOL LENGTH
//#define NDATA  (unsigned int) 5000
#define nWRITE (uint8_t)(14)
#define nREAD  (uint8_t)(18)

// ERP 42 PROTOCOL DEFAULT SETTING
#define S (uint8_t)(0x53)
#define T_ (uint8_t)(0x54)
#define X (uint8_t)(0x58)

#define AorM_Manual   (uint8_t)(0x00)
#define AorM_AutoMode (uint8_t)(0x01)

#define ESTOP_ON  (uint8_t)(0x01)
#define ESTOP_OFF (uint8_t)(0x00)

#define GEAR_FOWARD (uint8_t)(0x00)
#define GEAR_NEUTRAL (uint8_t)(0x01)
#define GEAR_BACKWORD (uint8_t)(0x02)

#define ETX0 (uint8_t)(0x0D)
#define ETX1 (uint8_t)(0x0A)

#define SPEED0 (uint8_t)(0x00) // range      0 ~ 200
#define SPEED1 (uint8_t)(0x00) // range      0 ~ 200
#define STEER0 (uint16_t)(0x00) // range  -2000 ~ 2000
#define STEER1 (uint8_t)(0x00) // range  -2000 ~ 2000
#define BRAKE  (uint8_t)(0x01) // range      1 ~ 200
#define ALIVE  (uint8_t)(0x00) // range       0 ~ 255
//-----------------------------------------------------------------------//
#define Ts_ERP  (double) 0.02
#define Fs_ERP  (int) 1/Ts_ERP
//#define NDATA  (unsigned int) 1500
// -----------------------------------------------------------------------------//
int cport_nr = 10,        /* /dev/ttyS0 (COM1 on windows) com3 => cport_nr = 2*/
bdrate = 115200;

char buf_Readdata[N_data][nREAD] = { 0.0, };
char buf_Writedata[N_data][nWRITE] = { 0.0, };

float buf_fReaddata[N_data][nREAD] = { 0.0, };
float buf_fWritedata[N_data][nWRITE] = { 0.0, };

char temp_read = NULL;
char* temp_pread;
char temp_write = NULL;
double time_intv = 20.0;
int steercmd = 1;
int speedcmd = 0;
int16_t  sinecmd = 0.0;
double sine = 0;
char mode[] = { '8','N','1',0 }; // RS232 serial protocol setting
int write_len = 0;
int read_len = 0;
char writebuffer[nWRITE] = { S,T_,X,AorM_AutoMode,ESTOP_OFF,GEAR_FOWARD,SPEED0,SPEED1,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1 };
char readbuffer[512] = { 0, };
unsigned int idx_alive = 0x00;
unsigned int idx_ndata = 0;

//------------------- SFF Variable ------------//
#define MAX_LIDAR_NUM		641			//500360

typedef struct FIELD
{
	double dis[MAX_LIDAR_NUM];
	double angle[MAX_LIDAR_NUM];
}FIELD
;

FIELD SFF;
FIELD BUF_SFF[N_data];
FIELD Max;

static double psirB;
static double sigSFF;
static double weightIFF;


#endif // !VARIABLE