// BNO055 IMU Sensoru icin Temel Kutuphane
//          v0.9.2 beta
//       author: dogu

#include "stm32f3xx_hal.h"
#include "bno055.h"
#include "main.h"
#include "i2c.h"
#include "stdint.h"

// BNO055'in proje generate ederken olusan handleri
extern I2C_HandleTypeDef hi2c1;
#define BNO055_I2C &hi2c1

// I2C adresi pg 90
#define BNO055_ADDRESS_WRITE 0x28
// Okuma ve yazma icin farkli adresler gerekli
#define BNO055_ADDRESS_READ 0x29

#define FILTER_SAMPLE 15 // 5'in katina gore ayarlanmistir


// pg 29
const uint8_t GyroPowerMode = FastPowerUpG;
const uint8_t GyroRange = GFS_2000DPS;
const uint8_t GyroBandwith = GBW_230Hz;

// pg 28
const uint8_t AccelRange = AFS_16G;
const uint8_t AccelMode = NormalA;
const uint8_t AccelBandwith = ABW_250Hz;

// pg 30
const uint8_t MagOperMode = EnhancedRegular;
const uint8_t MagPowMode = Normal;
const uint8_t MagOutputrate = MODR_30Hz;

// pg21
const uint8_t PWRMode = Normalpwr;
const uint8_t OPRMode = AMG;


float *xyz;


void BNO055_Config(void) {
    /** Alttaki veri alma fonksiyonunda
     * ilk parametre i2c handleri(pin configurasyonunda ayarliyoruz) (degiskenin adresi)
     * ikinci parametre BNO055'in i2c addressi datasheettde var,
     * 3. parametre BNO055'in hedef register,
     * 4. registerin bit boyutu. Iki durum var 8 ve 16bit. bno055'de cogulukla registerda 8 bit boyut olcak.
     * Boyut degeri i2c header da const deger olarak tanimlanmis I2C_MEMADD_SIZE_8BIT(1) veya I2C_MEMADD_SIZE_16BIT(2).
     * 5. gonderilecek ya da alinacak veri. (degiskenin adresi)
     * 6. parametre veri okunacak adim sayisi,
     * 7. islem icin maximum delay
     * **/


    // Acilis modunu secme
    // pg21 de bu modlarin detayi bulunuyor. ConfMode ile acip degerleri configure ederiz
    uint8_t mode = CONFIGMODE;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, 100);
    HAL_Delay(30);

    //  config yapmak ici config registerlarin oldugu page1'e gecme
    uint8_t page1 = 0x01;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_PAGE_ID, I2C_MEMADD_SIZE_8BIT, &page1, 1, 100);
    HAL_Delay(30);

    // Sensorlerin konfigurasyonu icin datasheetine gore gerekli deger olusturlup
    // regitera gonderilir. Assagida registerdaki degerlerin mantigi

    // https://ibb.co/gdBwRsZ

    // Accelometer config
    uint8_t accelConf = (AccelRange << 0) | (AccelMode << 5) | (AccelBandwith << 2);
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_ACC_CONFIG, I2C_MEMADD_SIZE_8BIT, &accelConf, 1, 100);
    HAL_Delay(30);

    // Gyroscope config
    uint8_t gyroConf = (GyroRange << 0) | (GyroBandwith << 3);
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_GYRO_CONFIG_0, I2C_MEMADD_SIZE_8BIT, &gyroConf, 1, 100);
    HAL_Delay(30);

    uint8_t gyroConf2 = GyroPowerMode << 0;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &gyroConf2, 1, 100);
    HAL_Delay(30);

    // Magnetometer config
    uint8_t magConf = (MagOperMode << 3) | (MagPowMode << 5) | (MagOutputrate << 0);
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_MAG_CONFIG, I2C_MEMADD_SIZE_8BIT, &magConf, 1, 100);
    HAL_Delay(30);

    // Page 0
    uint8_t page0 = 0x00;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_PAGE_ID, I2C_MEMADD_SIZE_8BIT, &page0, 1, 100);
    HAL_Delay(30);

    // Sensorleri aktiflestirmek icin operasyon modu secme
    uint8_t mainMode = OPRMode;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mainMode, 1, 100);
    HAL_Delay(30);


}

void BNO055_Calibration(void) {
    /** Sensor acildiginda calibre olmadan acilir config modunda acildiginda sensoru hareket ettrirerek
     * kalibre edilebilir. Sensor kalibre olup olmadigini 0x35 adresine kaydeder. 3 (1) degeri varsa kalibre edilmistir.
     * Gyro kalibresi icin sensor sabit tutulur, magnetometer icin havada 8 cizilir.
     * Ivme icin 45 derecelik acilarla sensorun sabit tutulmasi gerekir.
     *
     * pg 47
     * */

    uint8_t sys;
    uint8_t gyro;
    uint8_t acc;
    uint8_t mag;

    uint8_t calibration;

    int status = 0;
    while (status == 0) {
        HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_READ, BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT,
                         &calibration,
                         1, 100);


        // Registerda ikiser ikiser bitlere ayrilmistir. istenileni saga cekip 've'  baglayiciyla diger
        // degerler sifirlanir.
        sys = (calibration >> 6) & 0x03;

        gyro = (calibration >> 4) & 0x03;
        if (gyro == 3) {
            //          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }

        acc = (calibration >> 2) & 0x03;
        if (acc == 3){
            //        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
        }

        mag = calibration & 0x03;
        if (mag == 3) {
            //      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_GPIO_Port, GPIO_PIN_SET);
        }


        if (sys == 3 && gyro == 3 && acc == 3 && mag == 3) {
            status = 1;
        } else {
            status = 0;
        }

    }
}


// pg 48
void Get_Offsets(uint8_t *offsetData) {
    uint8_t mode = CONFIGMODE;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, 100);
    HAL_Delay(30);

    // Calibrasyon degerlerini sonradan hizlica kullanmak icin offset
    // BNO055_ACC_OFFSET_X_LSB itibaren 22 satir offset datasidir. Sonra bu degerleri tekrar atayarak kalibre edilmis olur
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_READ, BNO055_ACC_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, offsetData, 22, 100);


    uint8_t mode0 = OPRMode;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode0, 1, 100);
    HAL_Delay(30);
}

void Set_Offsets(uint8_t* offsetData) {
    uint8_t mode = CONFIGMODE;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, 100);
    HAL_Delay(30);


    uint8_t data[22];
    for (int i=0; i<22; i++){

        data[i]=offsetData[i];
    }

    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_ACC_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 22, 100);


    uint8_t mode2 = OPRMode;
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDRESS_WRITE, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode2, 1, 100);
    HAL_Delay(30);
}


void BNO055_Init(void) {


    uint8_t startingId;
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_READ, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &startingId, 1, 100);
    if (startingId != 0xA0) return; //Cihazin id si farkli

    BNO055_Config();

    // BNO055_Calibration();

}


//pg 33
float *Get_Accel_XYZ_Unfiltered(void) {
    /** Sensorde sayfa 33 den itibaren gosterildigi gibi her deger 2 bytedir(16 bit).
     * Her deger 8 bitlik iki parcaya ayrilip bu parcalar registerlara konur.
     * Mesala ivme sensorunun degeri binary olarak ikiye ayrilir: MSB ve LSB (binarynin buyuk kismi(sol) MSBdir)
     * Iki registerdan veri cekip bunlari 16 bitlik hale ceviririz.
     * Sonra MSB tarafini 8 bit sola kaydirip LSB ile 'veya' yardimiyla birlestirirz birlestiririz.
     * Bu saf degerimizi verir safi isleyip istenilen formata getirip kullaniriz.
     * **/

    // get accel data into xyz array
    uint8_t data[6] = {0};
    // Registerda acc_x_lsb den itibaren 6 registeri aliriz
    // Bu 6 si sirayla x y z nin lsb ve msb degerleri
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_READ, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

    int16_t x = 0, y = 0, z = 0;
    x = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    y = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    z = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);


    //   1 m/s^2 = 100 LSB
    xyz[0] = ((float) x) / 100.0;
    xyz[1] = ((float) y) / 100.0;
    xyz[2] = ((float) z) / 100.0;


    // atanilacak degisken basta pointer olarak baslatilmali sonradan bu pointera fonksiyon atanmalidir
    return xyz;
}

float *Get_Gyro_XYZ_Unfiltered(void) {
    // Accelometre ile mantigi ayni

    uint8_t data[6] = {0,1,2,3,4,5};
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_READ, BNO055_GYR_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);


    int16_t x = 0, y = 0, z = 0;

    x = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    y = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    z = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);

    // 1dps = 16 LSB
    xyz[0] = ((double) x) / 16.0;
    xyz[1] = ((double) y) / 16.0;
    xyz[2] = ((double) z) / 16.0;

    return xyz;


}

float *Get_Mag_XYZ_Unfiltered(void) {
    // Accelometre ile mantigi ayni
    uint8_t data[6] = {0};
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDRESS_READ, BNO055_MAG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

    int16_t x = 0, y = 0, z = 0;

    x = ((int16_t) data[0]) | (((int16_t) data[1]) << 8);
    y = ((int16_t) data[2]) | (((int16_t) data[3]) << 8);
    z = ((int16_t) data[4]) | (((int16_t) data[5]) << 8);

    // 1uT = 16 LSB
    xyz[0] = ((double) x) / 16.0;
    xyz[1] = ((double) y) / 16.0;
    xyz[2] = ((double) z) / 16.0;

    return xyz;
}


void sortArray(float *arry, int size) {
    int i, j, temp;
    for (i = 0; i < size; i++) {
        for (j = 0; j < size - 1; j++) {
            if (arry[j] > arry[j + 1]) {
                temp = arry[j];
                arry[j] = arry[j + 1];
                arry[j + 1] = temp;
            }
        }
    }
}

// Filter Sample daki kadar data cekilir.
// Data 5 li gruba ayrilir ve bu gruplarin medyanlari alinir.
// Daha sonra elde edilen medyanlarin ortalmasi alinir
float *Get_Accel_XYZ(void) {
    float matrixX[5];
    float matrixY[5];
    float matrixZ[5];
    float sumX = 0, sumY = 0, sumZ = 0;

    float *temp;


    for (int i = 0; i < FILTER_SAMPLE / 5; ++i) {

        for (int j = 0; j < 5; ++j) {
            temp = Get_Accel_XYZ_Unfiltered();
            matrixX[i] = temp[0];
            matrixY[i] = temp[1];
            matrixZ[i] = temp[2];
        }
        sortArray(matrixX, 5);
        sortArray(matrixY, 5);
        sortArray(matrixZ, 5);

        sumX += matrixX[2];
        sumY += matrixY[2];
        sumZ += matrixZ[2];
    }

    xyz[0] = sumX / (FILTER_SAMPLE / 5);
    xyz[1] = sumY / (FILTER_SAMPLE / 5);
    xyz[2] = sumZ / (FILTER_SAMPLE / 5);
    return xyz;
}

float *Get_Gyro_XYZ(void) {
    float matrixX[5];
    float matrixY[5];
    float matrixZ[5];
    float sumX = 0, sumY = 0, sumZ = 0;

    float *temp;


    for (int i = 0; i < FILTER_SAMPLE / 5; ++i) {

        for (int j = 0; j < 5; ++j) {
            temp = Get_Gyro_XYZ_Unfiltered();
            matrixX[i] = temp[0];
            matrixY[i] = temp[1];
            matrixZ[i] = temp[2];
        }
        sortArray(matrixX, 5);
        sortArray(matrixY, 5);
        sortArray(matrixZ, 5);

        sumX += matrixX[2];
        sumY += matrixY[2];
        sumZ += matrixZ[2];
    }

    xyz[0] = sumX / (FILTER_SAMPLE / 5);
    xyz[1] = sumY / (FILTER_SAMPLE / 5);
    xyz[2] = sumZ / (FILTER_SAMPLE / 5);
    return xyz;
}

float *Get_Mag_XYZ(void) {
    float matrixX[5];
    float matrixY[5];
    float matrixZ[5];
    float sumX = 0, sumY = 0, sumZ = 0;

    float *temp;


    for (int i = 0; i < FILTER_SAMPLE / 5; ++i) {

        for (int j = 0; j < 5; ++j) {
            temp = Get_Mag_XYZ_Unfiltered();
            matrixX[i] = temp[0];
            matrixY[i] = temp[1];
            matrixZ[i] = temp[2];
        }
        sortArray(matrixX, 5);
        sortArray(matrixY, 5);
        sortArray(matrixZ, 5);

        sumX += matrixX[2];
        sumY += matrixY[2];
        sumZ += matrixZ[2];
    }

    xyz[0] = sumX / (FILTER_SAMPLE / 5);
    xyz[1] = sumY / (FILTER_SAMPLE / 5);
    xyz[2] = sumZ / (FILTER_SAMPLE / 5);
    return xyz;

}

// ------------------------------------------------------------------
