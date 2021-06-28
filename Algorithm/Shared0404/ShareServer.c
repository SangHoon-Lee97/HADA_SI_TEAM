#ifndef SM_SERVER
#define SM_SERVER

#define _CRT_SECURE_NO_WARNINGS
#define RUN 2
#include "Include.h"
#include "variable.h"
#include "function.h"
#include "Time_Check.h"
#include "SharedMemory.h"

enum ERP_Protocol_Write {
    ewS = 0,
    eweT, ewX, ewAUTO, ewESTOP, ewGEAR, ewSPEED0, ewSPEED1, ewSTEER0, ewSTEER1, ewBRAKE, ewALIVE, ewETX0, ewETX1
};
enum ERP_Protocol_Read {
    erS = 0,
    erT, erX, erAUTO, erESTOP, erGEAR, erSPEED0, erSPEED1, erSTEER0, erSTEER1, erBRAKE, erENC0, erENC1, erENC2, erENC3, erALIVE, erETX0, erETX1
};
enum FLAG { COPLIETE = -1, STOP = 0, INIT = 1, START = 2 };

//extern double delta;
//extern double gamma;
//extern double Dist_R;
//extern double Dist_R2;
//extern double lambda;
//extern double delta_f;
//
//extern double buf_gamma[N_data];
//extern double buf_Dist_R[N_data];
//extern double buf_lambda[N_data];
//extern double buf_delta[N_data];
//extern double buf_delta_f[N_data];
//extern double GPS_waypoint_Lat[N_data];
//extern double GPS_waypoint_Lon[N_data];
//extern double GPS_waypoint_Lat_now;
//extern double GPS_waypoint_Lon_now;
//extern double buf_GPS_waypoint_Lat_now[N_data];
//extern double buf_GPS_waypoint_Lon_now[N_data];
//
//extern int    buf_Index[N_data];
//extern double buf_Raw_Lat[N_data];
//extern double buf_Raw_Lon[N_data];
//extern double buf_Raw_Roll[N_data];
//extern double buf_Raw_Pitch[N_data];
//extern double buf_Raw_Yaw[N_data];
//extern double buf_Raw_AccX[N_data];
//extern double buf_Raw_AccY[N_data];
//extern double buf_Raw_AccZ[N_data];
//extern double buf_Filtered_Lat[N_data];
//extern double buf_Filtered_Lon[N_data];
//extern double buf_Filtered_EVel[N_data];
//extern double buf_Filtered_NVel[N_data];
//
//// TM Conversion
//extern double buf_TM_Lat[N_data];
//extern double buf_TM_Lon[N_data];
//
//extern int    simcnt;
//extern int    idx_way;
//extern double Vec_BodyToWay[1][2];


# if RUN == 1
void main() {
    CreateClientXsens();
    CreateClientXsensFlag();

    //CreateClientNovA();
    //CreateClientNovAFlag();

    //CreateClientUblox();
    //CreateClientUbloxFlag();

    CreateServerNavi();
    CreateServerNaviFlag();

    //READ WayPoint True Value in TM Coordinate System.
    //ReadWayPoint_TM();


    //while (1)
    //{
    //    if (FlagRead(SERV_smdat_Flag) != STOP)
    //        break;
    //}

    dat_buf_Xsens_Flag.flag = 1;
   // dat_buf_Ublox_Flag.flag = 1;
    //dat_buf_NovA_Flag.flag = 1;

    FlagWrite(dat_buf_Xsens_Flag, CLI_smdat_Xsens_Flag);
    //FlagWrite(dat_buf_NovA_Flag, CLI_smdat_NovA_Flag);
   // FlagWrite(dat_buf_Ublox_Flag, CLI_smdat_Ublox_Flag);

    FILE* rFile = fopen("data_read.txt", "w");
    FILE* wFile = fopen("data_write.txt", "w");

    if (RS232_OpenComport(cport_nr, bdrate, mode, 0))
    {
        printf("Can not open comport\n");
        return(0);
    }

    for (int m = 0; m < MAX_LIDAR_NUM; m++)
    {
        SFF.angle[m] = (double)((0.25 * m) * PI / 180); // 640개 포인트에 대해 해상도 0.25 => 160도 => -80을 해줘서 범위를 -80 ~ 80으로 바꿔준다.
    }

    TimeInitialization();
    do
    {
        Buffer();

        if (simcnt % 5 == 0) {
            DataReadUblox(CLI_smdat_Ublox);
        }
        DataReadXsens(CLI_smdat_Xsens);

        //DataReadNovA(CLI_smdat_NovA);

        NavigationFilter();

        TMConversion();

        Pursuit_Guidance();

        WayPoint_Change();

        GenerateSFF();
        //// ERP42
        //if (simcnt % 2 == 0) {
        //    printf("Hello3");
        //    Command_ERP42();
        //    printf("Hello4");
        //}

        //dat_buf_Navi.SIM_count = SIM_count;
        //dat_buf_Navi.SIM_Time = SIM_Time;
        //dat_buf_Navi.Roll = roll;
        //dat_buf_Navi.Pitch = pitch;
        //dat_buf_Navi.Yaw = yaw;
        //dat_buf_Navi.Longtitude = longitude;
        //dat_buf_Navi.Latitude = latitude;
        //dat_buf_Navi.East_Vel = eX[2][0];
        //dat_buf_Navi.North_Vel = eX[3][0];
        //DataWrite(dat_buf_Navi);

        // SFF dis 값을 공유 메모리로 보내주기.
        memcpy(SERV_smdat_Navi->dis, SFF.dis, sizeof(SFF.dis));

        Idle_Time();
        simcnt++;

    } while (!CheckStop());

    //SaveData();

    ClosedClientXsens();
    ClosedClientXsensFlag();

    //ClosedClientNovA();
    //ClosedClientNovAFlag();

    //ClosedClientUblox();
    //ClosedClientUbloxFlag();

    //ClosedServerNavi();
    //ClosedServerNaviFlag();

}

void Buffer()
{
    index_imu = CLI_smdat_Xsens->SIM_count;
    roll = CLI_smdat_Xsens->euler_x;
    pitch = CLI_smdat_Xsens->euler_y;
    yaw = CLI_smdat_Xsens->euler_z;

    accX = CLI_smdat_Xsens->acc_x;
    accY = CLI_smdat_Xsens->acc_y;
    accZ = CLI_smdat_Xsens->acc_z;

    //index_gps = CLI_smdat_NovA->SIM_count;
    //latitude = CLI_smdat_NovA->LatDegree + CLI_smdat_NovA->LatMinute / 60.0;
    //longitude = CLI_smdat_NovA->LongDegree + CLI_smdat_NovA->LongMinute / 60.0;

    //index_gps = CLI_smdat_Ublox->SIM_count;
    //latitude = CLI_smdat_Ublox->LatDegree + CLI_smdat_Ublox->LatMinute / 60.0;
    //longitude = CLI_smdat_Ublox->LongDegree + CLI_smdat_Ublox->LongMinute / 60.0;

    buf_Raw_Lat[simcnt] = latitude;
    buf_Raw_Lon[simcnt] = longitude;
    buf_Raw_Roll[simcnt] = roll;
    buf_Raw_Pitch[simcnt] = pitch;
    buf_Raw_Yaw[simcnt] = yaw;
    buf_Raw_AccX[simcnt] = accX;
    buf_Raw_AccY[simcnt] = accY;
    buf_Raw_AccZ[simcnt] = accZ;

    //printf("menwrite%d %f %f %f %f %f %d %f %f \n", index_imu, roll, pitch, yaw, accX, accY, index_gps, latitude, longitude);
    //printf("menwrite%d %f %f\n", index_gps, latitude, longitude);
}

void   NavigationFilter(void)
{
    int i, j, k;

    U_acc[0][0] = CLI_smdat_Xsens->acc_x;
    U_acc[1][0] = CLI_smdat_Xsens->acc_y;
    U_acc[2][0] = CLI_smdat_Xsens->acc_z;
    //printf("filter%d %f %f %f %f %f %d %f %f \n", index_imu, roll, pitch, yaw, accX, accY, index_gps, latitude, longitude);

    if ((36 < CLI_smdat_NovA->LatDegree) && (CLI_smdat_NovA->LatDegree < 37) && (129 < CLI_smdat_NovA->LongDegree) && (CLI_smdat_NovA->LongDegree < 130))
    {
        Ygps[0][0] = (CLI_smdat_NovA->LatDegree - LAT0) * R0;
        Ygps[1][0] = (CLI_smdat_NovA->LongDegree - LON0) * R0;
        //printf("\nYgps1: %f\tYgps2: %f\n", Ygps[0][0], Ygps[1][0]);
        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                A_eX[i][j] = 0;
                for (k = 0; k < 4; k++) {
                    A_eX[i][j] = A_bar[i][k] * eX[k][j];
                    //printf("A_eX: %d %f \n", simcnt, A_eX[i][0]);
                }
            }
        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                //B_eX[i][j] = 0;
                for (k = 0; k < 2; k++) {
                    B_acc[i][j] = B_bar[i][k] * U_acc[k][j];
                    //printf("B_acc: %d %f \n", simcnt, B_acc[i][j]);
                }
            }
        for (i = 0; i < 2; i++)
            for (j = 0; j < 1; j++) {
                C_eX[i][j] = 0;
                for (k = 0; k < 4; k++) {
                    C_eX[i][j] = C[i][k] * eX[k][j];
                    //printf("C_eX: %d %f \n", simcnt, C_eX[i][j]);
                }
            }
        for (i = 0; i < 2; i++) {
            _Ygps[i][0] = Ygps[i][0] - C_eX[i][0];
            //printf("_Ygps: %d %f \n", simcnt, _Ygps[i][0]);
        }

        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                Kf_gps[i][j] = 0;
                for (k = 0; k < 2; k++) {
                    Kf_gps[i][j] = Kf_inf[i][k] * _Ygps[k][j];
                    //printf("Kf_gps: %d %f \n", simcnt, Kf_gps[i][j]);
                }
            }

        for (i = 0; i < 4; i++)
        {
            eX[i][0] = A_eX[i][0] + B_acc[i][0] + Kf_gps[i][0];
            //printf("if:  %d %f \n", simcnt, eX[i][0]);
            //printf("eX: %d %f \n", simcnt, eX[i][0]);
        }

    }
    else
    {

        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                A_eX[i][j] = 0;
                for (k = 0; k < 4; k++)
                    A_eX[i][j] = A_bar[i][k] * eX[k][j];
            }
        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                B_acc[i][j] = 0;
                for (k = 0; k < 2; k++)
                    B_acc[i][j] = B_bar[i][k] * U_acc[k][j];
            }
        for (i = 0; i < 4; i++)
            eX[i][0] = A_eX[i][0] + B_acc[i][0];

    }

    buf_Filtered_Lat[simcnt] = eX[0][0]; //
    buf_Filtered_Lon[simcnt] = eX[1][0]; //
    buf_Filtered_EVel[simcnt] = eX[2][0]; //
    buf_Filtered_NVel[simcnt] = eX[3][0]; //
}

void TMConversion(void) {

    if ((36 < CLI_smdat_NovA->LatDegree) && (CLI_smdat_NovA->LatDegree < 37) && (129 < CLI_smdat_NovA->LongDegree) && (CLI_smdat_NovA->LongDegree < 130)) {
        Curr_Lat = CLI_smdat_Ublox->LatDegree;
        Curr_Lon = CLI_smdat_Ublox->LongDegree;
    }

    //Curr_Lat = buf_Filtered_Lat[simcnt];
    //Curr_Lon = buf_Filtered_Lon[simcnt];

    T = pow(tan(Curr_Lat * DEG2RAD), 2);
    C_ = pow(Ecc_1, 2) / (1 - pow(Ecc_1, 2)) * pow(cos(Curr_Lat * DEG2RAD), 2);
    A = ((Curr_Lon * DEG2RAD - Lon_std * DEG2RAD)) * cos(Curr_Lat * DEG2RAD);
    N = Long_R * sqrt(1 - Ecc_1 * pow(sin(Curr_Lat * DEG2RAD), 2));

    M = (Long_R * ((1 - Ecc_1 / 4 - 3 * pow(Ecc_1, 2) / 64 - 5 * pow(Ecc_1, 3) / 256) * Curr_Lat * DEG2RAD) - (3 * Ecc_1 / 8 + 3 * pow(Ecc_1, 2) / 32 + 45 * pow(Ecc_1, 3) / 1024) * sin(2 * Curr_Lat * DEG2RAD) + (15 * pow(Ecc_1, 2) / 256 + 45 * pow(Ecc_1, 3) / 1024) * sin(4 * Curr_Lat * DEG2RAD) - 35 * pow(Ecc_1, 3) / 3072 * sin(6 * Curr_Lat * DEG2RAD));

    TM_Lat = dx + K_scale * (M - M0 + N * tan(Curr_Lat * DEG2RAD)) * (pow(A, 2) / 2 + pow(A, 4) / 24 * (5 - T + 9 * C_ + 4 * pow(C_, 2)) + pow(A, 6) / 720 * (61 - 58 * T + pow(T, 2) + 600 * C_ - 330 * Ecc_2));
    TM_Lon = dy + K_scale * N * (A + pow(A, 3) / 6 * (1 - T + C_) + pow(A, 5) / 120 * (5 - 18 * T + pow(T, 2) + 72 * C_ - 58 * Ecc_2));

    buf_TM_Lat[simcnt] = TM_Lat;
    buf_TM_Lon[simcnt] = TM_Lon;

}


void Pursuit_Guidance(void) {
    gamma = 90 - buf_Raw_Yaw[simcnt];
    if (gamma > 180)
        gamma = gamma - 360;
    gamma = gamma * DEG2RAD;
    buf_gamma[simcnt] = gamma;

    // Distance and Lambda Calculation
    Vec_BodyToWay[1] = GPS_waypoint_Lat[simcnt] - buf_TM_Lat[simcnt];
    Vec_BodyToWay[2] = GPS_waypoint_Lon[simcnt] - buf_TM_Lon[simcnt];

    Dist_R = sqrt(pow(Vec_BodyToWay[1], 2)) + sqrt(pow(Vec_BodyToWay[2], 2));
    buf_Dist_R[simcnt] = Dist_R;

    lambda = atan2(Vec_BodyToWay[1], Vec_BodyToWay[2]);
    buf_lambda[simcnt] = lambda;

    // Delta Calculation
    delta = gamma - lambda;
    if (delta > PI)
        delta = delta - 2 * PI;
    else if (delta < -PI)
        delta = delta + 2 * PI;
    buf_delta[simcnt] = delta;

    // Delta_f Calculation
    delta_f = (-2 * Body_L * K_GLOB * sin(delta)) / Dist_R;
    delta_f = RAD2DEG * delta_f;
    if (delta_f > 10)
        delta_f = 10;
    else if (delta_f < -10)
        delta_f = -10;
    buf_delta_f[simcnt] = delta_f;

}

//void ReadWayPoint_TM(void) {
//
//    double num1;
//    double num2;
//    char str[500];
//
//    FILE* file = fopen("Waypoint.txt", "r");
//
//    if (file == NULL) {
//        printf("파일열기 실패\n");
//        return 1;
//    }
//
//    //fprintf(file, "%d %d %s \n", num1, num2, "입력되었습니다.");
//    while (!feof(file)) {
//        fscanf(file, "%f %f %s \n", &num1, &num2, str);
//        printf("%f %f %s \n", num1, num2, str);
//        
//    }
//    fclose(file);
//
//}

void WayPoint_Change(void) {
    if (Dist_R < 3) {
        GPS_waypoint_Lat_now = GPS_waypoint_Lat[idx_way];
        GPS_waypoint_Lon_now = GPS_waypoint_Lon[idx_way];
        idx_way++;
    }
}

void Command_ERP42(void) {
    read_len = RS232_PollComport(cport_nr, readbuffer, sizeof(readbuffer)); // RS232 data 읽어오기, output은 읽어온 데이터 길이
       //printf("read_len : %d\n", read_len);

    if (read_len > 0)
    {
        readbuffer[nREAD] = 0;   /* always put a "null" at the end of a string! */
        //printf("readbuffer = ");
        for (int e = 0; e < nREAD; e++)
        {
            printf("%d ", (char*)readbuffer[e]);
            temp_read = readbuffer[e];
            buf_Readdata[idx_ndata][e] = temp_read;
            temp_read = NULL;
        }
        //printf("\n");
    }

    //write data
    write_len = RS232_SendBuf(cport_nr, &writebuffer, sizeof(writebuffer));
    //printf("writebuffer = ");
    for (int e = 0; e < nWRITE; e++) {
        //printf("%d ", writebuffer[e]);
        temp_write = writebuffer[e];
        buf_Writedata[idx_ndata][e] = temp_write;
        temp_write = NULL;
    }
    //printf("\n");

    if (speedcmd) {
        if (simTime > 0 && simTime < time_intv) {
            writebuffer[ewSPEED0] = 0x00;  //0 ~200 0x00 ~ 0xc8
            writebuffer[ewSPEED1] = 0x96;  //
        }
        else if (simTime >= time_intv && simTime < RUNTIME)
        {
            writebuffer[ewSPEED0] = 0x00;
            writebuffer[ewSPEED1] = 0x00;
            writebuffer[ewBRAKE] = 0x28;
        }
    }

    if (steercmd) {
        delta_f = 20;
        sinecmd = (2000 / 28) * delta_f;
        // if (sinecmd > 0)
        writebuffer[ewSTEER0] = (sinecmd & 0xff00) >> 8;   //
        writebuffer[ewSTEER1] = (sinecmd & 0xff);
    }

    writebuffer[ewALIVE] = idx_alive; // loop마다 1씩 증가해서 오류 잡기 alive
    idx_alive++; idx_alive %= 0xff;           // 최대 255까지만 해서 버퍼 오버플로우 안나게
    idx_ndata++;

}

void GenerateSFF(void)
{
    sigSFF = 0.4;
    for (int i = 0; i < MAX_LIDAR_NUM; i++)
    {
        //SFF.angle[i] = cur_obstacle.raw_angle[i];
        SFF.dis[i] = exp(-pow((SFF.angle[i] - delta_f), 2) / (2.0 * sigSFF * sigSFF));
        //printf("SFF: %f\n", SFF.dis[i]);
    }
}


//void SaveData()
//{
   //pFile = fopen("RawData.csv", "wt");

   //if (pFile == NULL) {
   //   printf("안됨..\n");
   //   return 1;
   //}

   //for (int r = 0; r < N_data; r++) {
   //   fprintf(pFile, "%d, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n",
   //      (int)   buf_Index[r]   , (double)buf_Raw_Lat[r]  , (double)buf_Raw_Lon[r],
   //      (double)buf_Raw_Roll[r], (double)buf_Raw_Pitch[r], (double)buf_Raw_Yaw[r],
   //      (double)buf_Raw_AccX[r], (double)buf_Raw_AccY[r],   (double)buf_Raw_AccZ[r]);

   //}
   //printf("Raw data Successful\n");
   ////

   //pFile = fopen("FilterData.csv", "wt");

   //if (pFile == NULL) {
   //   printf("안됨..\n");
   //   return 1;
   //}

   //for (int r = 0; r < N_data; r++) {
   //   fprintf(pFile, "%d, %.7f, %.7f, %.7f, %.7f\n",
   //      (int)buf_Index[r], (double)buf_Filtered_Lat[r], (double)buf_Filtered_Lon[r], (double)buf_Filtered_EVel[r], (double)buf_Filtered_NVel[r]);
   //}

   //printf("Filtered Data Successful.\n" );

   //fclose(pFile);
   //printf("File storing done");
//}

# elif RUN == 2

void main() {

    CreateClientXsens();
    CreateClientXsensFlag();

    //CreateClientNovA();
    //CreateClientNovAFlag();
    
    CreateClientUblox();
    CreateClientUbloxFlag();

    //CreateServerNavi();         // Solution with Navigation Info 
    //CreateServerNaviFlag();

    //READ WayPoint True Value in TM Coordinate System.
    //ReadWayPoint_TM();

    while (1)
    {
        /*if (FlagRead(SERV_smdat_Flag) != STOP)
            break;*/
        if (FlagRead(CLI_smdat_Xsens_Flag) != STOP)
            break;
    }

    //_getch();

    dat_buf_Xsens_Flag.flag = 1;
    dat_buf_Ublox_Flag.flag = 1;
    //dat_buf_NovA_Flag.flag = 1;

    FlagWrite(dat_buf_Xsens_Flag, CLI_smdat_Xsens_Flag);
    FlagWrite(dat_buf_Ublox_Flag, CLI_smdat_Ublox_Flag);

    //FlagWrite(dat_buf_NovA_Flag, CLI_smdat_NovA_Flag);

    FILE* rFile = fopen("data_read.txt", "w");
    FILE* wFile = fopen("data_write.txt", "w");

    if (RS232_OpenComport(cport_nr, bdrate, mode, 0))
    {
        printf("Can not open comport\n");
        return(0);
    }

    TimeInitialization();
    do
    {
        if (simcnt % 5 == 0) {
            Buffer_Ublox();
            DataReadUblox(CLI_smdat_Ublox);  // Check Whether data is fine
        }
        Buffer_Xsens();
        DataReadXsens(CLI_smdat_Xsens);      // Check Whether data is fine

        //DataReadNovA(CLI_smdat_NovA);

        NavigationFilter();
        TMConversion();

        Pursuit_Guidance();
        
        WayPoint_Change();

        if (simcnt % 2 == 0) {
            Command_ERP42();
        }

        //dat_buf_Navi.SIM_count = SIM_count;
        //dat_buf_Navi.SIM_Time = SIM_Time;
        //dat_buf_Navi.Roll = roll;
        //dat_buf_Navi.Pitch = pitch;
        //dat_buf_Navi.Yaw = yaw;
        //dat_buf_Navi.Longtitude = longitude;
        //dat_buf_Navi.Latitude = latitude;
        //dat_buf_Navi.East_Vel = eX[2][0];
        //dat_buf_Navi.North_Vel = eX[3][0];

        //DataWrite(dat_buf_Navi);

        Idle_Time();
        simcnt++;

    } while (!CheckStop());

    SaveData();

    ClosedClientXsens();
    ClosedClientXsensFlag();

    //ClosedClientNovA();
    //ClosedClientNovAFlag();

    ClosedClientUblox();
    ClosedClientUbloxFlag();

    //ClosedServerNavi();
    //ClosedServerNaviFlag();
}


void Buffer_Xsens()
{
    //index_imu = CLI_smdat_Xsens->SIM_count;
    roll = CLI_smdat_Xsens->euler_x;
    pitch = CLI_smdat_Xsens->euler_y;
    yaw = CLI_smdat_Xsens->euler_z;

    accX = CLI_smdat_Xsens->acc_x;
    accY = CLI_smdat_Xsens->acc_y;
    accZ = CLI_smdat_Xsens->acc_z;

    //index_gps = CLI_smdat_NovA->SIM_count;
    //latitude = CLI_smdat_NovA->LatDegree + CLI_smdat_NovA->LatMinute / 60.0;
    //longitude = CLI_smdat_NovA->LongDegree + CLI_smdat_NovA->LongMinute / 60.0;

    buf_Raw_Roll[simcnt] = roll;
    buf_Raw_Pitch[simcnt] = pitch;
    buf_Raw_Yaw[simcnt] = yaw;
    buf_Raw_AccX[simcnt] = accX;
    buf_Raw_AccY[simcnt] = accY;
    buf_Raw_AccZ[simcnt] = accZ;

    //printf("menwrite%d %f %f %f %f %f %d %f %f \n", index_imu, roll, pitch, yaw, accX, accY, index_gps, latitude, longitude);

    //printf("menwrite%d %f %f\n", index_gps, latitude, longitude);
}

void Buffer_Ublox()
{
    //index_gps = CLI_smdat_NovA->SIM_count;
    //latitude = CLI_smdat_NovA->LatDegree + CLI_smdat_NovA->LatMinute / 60.0;
    //longitude = CLI_smdat_NovA->LongDegree + CLI_smdat_NovA->LongMinute / 60.0;

    //index_gps = CLI_smdat_Ublox->SIM_count;
    latitude = CLI_smdat_Ublox->LatDegree + CLI_smdat_Ublox->LatMinute / 60.0;
    longitude = CLI_smdat_Ublox->LongDegree + CLI_smdat_Ublox->LongMinute / 60.0;

    buf_Raw_Lat[simcnt] = latitude;
    buf_Raw_Lon[simcnt] = longitude;

    //printf("menwrite%d %f %f %f %f %f %d %f %f \n", index_imu, roll, pitch, yaw, accX, accY, index_gps, latitude, longitude);

    //printf("menwrite%d %f %f\n", index_gps, latitude, longitude);
}



void   NavigationFilter(void)
{
    int i, j, k;

    U_acc[0][0] = CLI_smdat_Xsens->acc_x;
    U_acc[1][0] = CLI_smdat_Xsens->acc_y;
    U_acc[2][0] = CLI_smdat_Xsens->acc_z;
    //printf("filter%d %f %f %f %f %f %d %f %f \n", index_imu, roll, pitch, yaw, accX, accY, index_gps, latitude, longitude);

    if ((36 < CLI_smdat_Ublox->LatDegree) && (CLI_smdat_Ublox->LatDegree < 37) && (129 < CLI_smdat_Ublox->LongDegree) && (CLI_smdat_Ublox->LongDegree < 130))
    {
        Ygps[0][0] = (CLI_smdat_Ublox->LatDegree - LAT0) * R0;
        Ygps[1][0] = (CLI_smdat_Ublox->LongDegree - LON0) * R0;
        //printf("\nYgps1: %f\tYgps2: %f\n", Ygps[0][0], Ygps[1][0]);

        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                A_eX[i][j] = 0;
                for (k = 0; k < 4; k++) {
                    A_eX[i][j] = A_bar[i][k] * eX[k][j];
                    //printf("A_eX: %d %f \n", simcnt, A_eX[i][0]);
                }
            }
        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                //B_eX[i][j] = 0;
                for (k = 0; k < 2; k++) {
                    B_acc[i][j] = B_bar[i][k] * U_acc[k][j];
                    //printf("B_acc: %d %f \n", simcnt, B_acc[i][j]);
                }
            }
        for (i = 0; i < 2; i++)
            for (j = 0; j < 1; j++) {
                C_eX[i][j] = 0;
                for (k = 0; k < 4; k++) {
                    C_eX[i][j] = C[i][k] * eX[k][j];
                    //printf("C_eX: %d %f \n", simcnt, C_eX[i][j]);
                }
            }
        for (i = 0; i < 2; i++) {
            _Ygps[i][0] = Ygps[i][0] - C_eX[i][0];
            //printf("_Ygps: %d %f \n", simcnt, _Ygps[i][0]);
        }

        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                Kf_gps[i][j] = 0;
                for (k = 0; k < 2; k++) {
                    Kf_gps[i][j] = Kf_inf[i][k] * _Ygps[k][j];
                    //printf("Kf_gps: %d %f \n", simcnt, Kf_gps[i][j]);
                }
            }

        for (i = 0; i < 4; i++)
        {
            eX[i][0] = A_eX[i][0] + B_acc[i][0] + Kf_gps[i][0];
            //printf("if:  %d %f \n", simcnt, eX[i][0]);
            //printf("eX: %d %f \n", simcnt, eX[i][0]);
        }
    }
    else
    {

        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                A_eX[i][j] = 0;
                for (k = 0; k < 4; k++)
                    A_eX[i][j] = A_bar[i][k] * eX[k][j];
            }
        for (i = 0; i < 4; i++)
            for (j = 0; j < 1; j++) {
                B_acc[i][j] = 0;
                for (k = 0; k < 2; k++)
                    B_acc[i][j] = B_bar[i][k] * U_acc[k][j];
            }
        for (i = 0; i < 4; i++)
            eX[i][0] = A_eX[i][0] + B_acc[i][0];

    }

    buf_Filtered_Lat[simcnt] = eX[0][0]; //
    buf_Filtered_Lon[simcnt] = eX[1][0]; //
    buf_Filtered_EVel[simcnt] = eX[2][0]; //
    buf_Filtered_NVel[simcnt] = eX[3][0]; //
}

void TMConversion(void) {

    if ((36 < CLI_smdat_Ublox->LatDegree) && (CLI_smdat_Ublox->LatDegree < 37) && (129 < CLI_smdat_Ublox->LongDegree) && (CLI_smdat_Ublox->LongDegree < 130)) {
        Curr_Lat = CLI_smdat_Ublox->LatDegree;
        Curr_Lon = CLI_smdat_Ublox->LongDegree;
    }

    //Curr_Lat = buf_Filtered_Lat[simcnt];
    //Curr_Lon = buf_Filtered_Lon[simcnt];

    T = pow(tan(Curr_Lat * DEG2RAD), 2);
    C_ = pow(Ecc_1, 2) / (1 - pow(Ecc_1, 2)) * pow(cos(Curr_Lat * DEG2RAD), 2);
    A = ((Curr_Lon * DEG2RAD - Lon_std * DEG2RAD)) * cos(Curr_Lat * DEG2RAD);
    N = Long_R * sqrt(1 - Ecc_1 * pow(sin(Curr_Lat * DEG2RAD), 2));

    M = (Long_R * ((1 - Ecc_1 / 4 - 3 * pow(Ecc_1, 2) / 64 - 5 * pow(Ecc_1, 3) / 256) * Curr_Lat * DEG2RAD) - (3 * Ecc_1 / 8 + 3 * pow(Ecc_1, 2) / 32 + 45 * pow(Ecc_1, 3) / 1024) * sin(2 * Curr_Lat * DEG2RAD) + (15 * pow(Ecc_1, 2) / 256 + 45 * pow(Ecc_1, 3) / 1024) * sin(4 * Curr_Lat * DEG2RAD) - 35 * pow(Ecc_1, 3) / 3072 * sin(6 * Curr_Lat * DEG2RAD));

    TM_Lat = dx + K_scale * (M - M0 + N * tan(Curr_Lat * DEG2RAD)) * (pow(A, 2) / 2 + pow(A, 4) / 24 * (5 - T + 9 * C_ + 4 * pow(C_, 2)) + pow(A, 6) / 720 * (61 - 58 * T + pow(T, 2) + 600 * C_ - 330 * Ecc_2));
    TM_Lon = dy + K_scale * N * (A + pow(A, 3) / 6 * (1 - T + C_) + pow(A, 5) / 120 * (5 - 18 * T + pow(T, 2) + 72 * C_ - 58 * Ecc_2));

    buf_TM_Lat[simcnt] = TM_Lat;
    buf_TM_Lon[simcnt] = TM_Lon;

}


void Pursuit_Guidance(void) {

    gamma = buf_Raw_Yaw[simcnt];
    if (gamma > 180 * DEG2RAD)
        gamma = gamma - 180 * DEG2RAD;
    else if (gamma < -180 * DEG2RAD)
        gamma = gamma + 180 * DEG2RAD;


    // Distance and Lambda Calculation
    Vec_BodyToWay[0][0] = GPS_waypoint_Lat_now - buf_TM_Lat[simcnt];
    Vec_BodyToWay[0][1] = GPS_waypoint_Lon_now - buf_TM_Lon[simcnt];

    Dist_R = sqrt(pow(Vec_BodyToWay[0][0], 2)) + sqrt(pow(Vec_BodyToWay[0][1], 2));
    lambda = atan2(Vec_BodyToWay[0][1], Vec_BodyToWay[0][0]);

    // Delta Calculation
    delta = gamma - lambda;
    if (delta > 180 * DEG2RAD)
        delta = delta - 180 * DEG2RAD;
    else if (delta < -180 * DEG2RAD)
        delta = delta + 180 * DEG2RAD;
    
    // Delta_f Calculation
    delta_f = (-2 * Body_L * K_GLOB * sin(delta)) / Dist_R;
    delta_f = RAD2DEG * delta_f;
    if (delta_f > MAX_STEER)
        delta_f = MAX_STEER;
    else if (delta_f < -MAX_STEER)
        delta_f = -MAX_STEER;
    printf("gamma : %f lambda : %f delfa_f %f delfa_f %f \n", gamma, lambda, delta, delta_f);


    buf_delta[simcnt] = delta;
    buf_lambda[simcnt] = lambda;
    buf_Dist_R[simcnt] = Dist_R;
    buf_gamma[simcnt] = gamma;
    buf_delta_f[simcnt] = delta_f;
    buf_GPS_waypoint_Lat_now[idx_ndata] = GPS_waypoint_Lat_now;
    buf_GPS_waypoint_Lon_now[idx_ndata] = GPS_waypoint_Lon_now;
}

void WayPoint_Change(void) {
    
    if (Dist_R < MIN_DIST) {
        GPS_waypoint_Lat_now = GPS_waypoint_Lat[idx_way];
        GPS_waypoint_Lon_now = GPS_waypoint_Lon[idx_way];
        idx_way++;
    }
    
    /*printf("\nGPS_WAYPOINT_Lat : %f \n", GPS_waypoint_Lat_now);
    printf("GPS_WAYPOINT_Lon : %f \n", GPS_waypoint_Lon_now);*/
}

void Command_ERP42(void) {
    read_len = RS232_PollComport(cport_nr, readbuffer, sizeof(readbuffer)); // RS232 data 읽어오기, output은 읽어온 데이터 길이
       //printf("read_len : %d\n", read_len);

    if (read_len > 0)
    {
        readbuffer[nREAD] = 0;   /* always put a "null" at the end of a string! */
        //printf("readbuffer = ");
        for (int e = 0; e < nREAD; e++)
        {
            printf("%d ", (char*)readbuffer[e]);
            temp_read = readbuffer[e];
            buf_Readdata[idx_ndata][e] = temp_read;
            temp_read = NULL;
        }
        //printf("\n");
    }

    //write data
    write_len = RS232_SendBuf(cport_nr, &writebuffer, sizeof(writebuffer));
    //printf("writebuffer = ");
    for (int e = 0; e < nWRITE; e++) {
        //printf("%d ", writebuffer[e]);
        temp_write = writebuffer[e];
        buf_Writedata[idx_ndata][e] = temp_write;
        temp_write = NULL;
    }
    //printf("\n");

    if (speedcmd) {
        if (simTime > 0 && simTime < time_intv) {
            writebuffer[ewSPEED0] = 0x00;  //0 ~200 0x00 ~ 0xc8
            writebuffer[ewSPEED1] = 0x96;  //
        }
        else if (simTime >= time_intv && simTime < RUNTIME)
        {
            writebuffer[ewSPEED0] = 0x00;
            writebuffer[ewSPEED1] = 0x00;
            writebuffer[ewBRAKE] = 0x28;
        }
    }

    if (steercmd) {
        sinecmd = (2000 / 28) * delta_f;
        // if (sinecmd > 0)
        writebuffer[ewSTEER0] = (sinecmd & 0xff00) >> 8;   //
        writebuffer[ewSTEER1] = (sinecmd & 0xff);
    }

    writebuffer[ewALIVE] = idx_alive; // loop마다 1씩 증가해서 오류 잡기 alive
    idx_alive++; idx_alive %= 0xff;           // 최대 255까지만 해서 버퍼 오버플로우 안나게
    idx_ndata++;

}

void SaveData()
{
   pFile = fopen("RawData.csv", "wt");

   if (pFile == NULL) {
      printf("안됨..\n");
      return 1;
   }

   for (int r = 0; r < N_data; r++) {
      fprintf(pFile, "%d, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n",
         (int)   buf_Index[r]   , (double)buf_Raw_Lat[r]  , (double)buf_Raw_Lon[r],
         (double)buf_Raw_Roll[r], (double)buf_Raw_Pitch[r], (double)buf_Raw_Yaw[r],
         (double)buf_Raw_AccX[r], (double)buf_Raw_AccY[r],   (double)buf_Raw_AccZ[r]);

   }
   printf("Raw data Successful\n");
   //

   pFile = fopen("FilterData.csv", "wt");

   if (pFile == NULL) {
      printf("안됨..\n");
      return 1;
   }

   for (int r = 0; r < N_data; r++) {
      fprintf(pFile, "%d, %.7f, %.7f, %.7f, %.7f\n",
         (int)buf_Index[r], (double)buf_Filtered_Lat[r], (double)buf_Filtered_Lon[r], (double)buf_Filtered_EVel[r], (double)buf_Filtered_NVel[r]);
   }

   printf("Filtered Data Successful.\n" );

   fclose(pFile);
   printf("File storing done");
}

//void ReadWayPoint_TM(void) {
//
//    double num1;
//    double num2;
//    char str[500];
//
//    FILE* file = fopen("Waypoint.txt", "r");
//
//    if (file == NULL) {
//        printf("파일열기 실패\n");
//        return 1;
//    }
//
//    //fprintf(file, "%d %d %s \n", num1, num2, "입력되었습니다.");
//    while (!feof(file)) {
//        fscanf(file, "%f %f %s \n", &num1, &num2, str);
//        printf("%f %f %s \n", num1, num2, str);
//        
//    }
//    fclose(file);
//
//}


#endif

#endif
