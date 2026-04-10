/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ov7670.h"
#include "LED.h"
#include "SD_Card.h"
#include "ILI9341.h"
#include "w9825g6kh.h"
#include "pca9685.h"
#include "math.h"
#include "bno055_stm32.h"
#include "bno055.h"
#include "ai_platform.h"
#include "network_1752296348456.h"
#include "network_1752296348456_data.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint32_t bus_type;
bno055_vector_t v;
bno055_vector_t d;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUS_WIDTH sizeof(bus_type)
#define SDRAM_BASE_ADDR   0xC0000000
#define SDRAM_SIZE_BYTES  (32 * 1024 * 1024)
#define TEST_CHUNK_SIZE   (1024 * 1024)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVOMIN  125 // ~0.6ms (Góc 0)
#define SERVOMAX  490 // ~2.4ms (Góc 180)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

DMA2D_HandleTypeDef hdma2d;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
W9825G6KH_Context_t SDRAM_Config;
int32_t ret;
//uint32_t *pMem = (uint32_t *)0xC0000000; // Địa chỉ Bank 1

float rl_action[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float smoothed_action[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float scale_array[6] = {0.05f, 0.05f, 0.03f, 0.0f, 0.04f, 0.04f}; // res_dx_l, res_dx_r, res_dy, res_dz(OFF), res_lift_l, res_lift_r
const float EMA_ALPHA = 0.85f;
const float STD_Z = 0.265f;
const float STD_Y = 0.01f;

// ------ PPO RL OBSERVATION PIPELINE -----
float history_buffer[126] = {0.0f}; 
float prev_raw_action[6] = {0.0f}; 
float cpg_phase = 0.0f;
uint32_t last_inference_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DCMI_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM13_Init(void);
static void MX_DMA2D_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */
extern const uint8_t LOGO[];
extern const uint32_t LOGO_size;
uint8_t my_i2c_devices[10];
uint8_t dev_count = 0;
//-------------IMU------------------
int goc_ht[2]={0,0}, goc_trc[2]={0,0}, delta_goc[2]={0,0}, goc_tong[2]={0,0};
uint8_t Ks=2;
float X,Y,a;
extern uint8_t UserRxBufferFS[];
double l_a[5], r_a[5];
float rad2deg=180/M_PI;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _close(int file) { return -1; }
int _fstat(int file) { return -1; }
int _isatty(int file) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char *ptr, int len) { return 0; }
int _write(int file, char *ptr, int len) { return len; }
int _getpid(void) {return 1;}
int _kill(int pid, int sig) {return -1;}
 uint32_t t;
 int32_t delta_t;
 float f=0;
extern const uint8_t LOGO[];
extern const uint32_t LOGO_size;
extern __attribute__((section(".D1"))) ai_i8 data_in_1[AI_NETWORK_1752296348456_IN_1_SIZE_BYTES];
extern __attribute__((section(".D1"))) ai_i8 data_out_1[AI_NETWORK_1752296348456_OUT_1_SIZE_BYTES];
extern __attribute__((section(".D1"))) uint8_t frame_192x192[DST_WIDTH * DST_HEIGHT * 2];
const double L3 = 0.08;
const double L4 = 0.15;
const double L5 = 0.065;
const double ALPHA = 0.6;

extern int class_id;
extern float score;
extern float *check;
extern int box_w ;
extern int box_h;
extern float cx ;
extern float cy;
int cnt=0;
extern float x_ofs, y_ofs;
int ai_x[10], ai_y[10], ai_w[10], ai_h[10];
float ai_score[10];
char label[32];
extern float x_ofs, y_ofs;
int aaa[16]={90},bbb[16]={90};
extern struct
{
    /* HAL peripheral handlers */
    DCMI_HandleTypeDef  *hdcmi;
    I2C_HandleTypeDef   *hi2c;
    TIM_HandleTypeDef   *htim;
    uint32_t            tim_ch;
    /* Requested mode */
    uint32_t            mode;
    /* Address of the buffer */
    volatile uint32_t   buffer_addr[2];
    /* Image line counter */
    volatile uint32_t   lineCnt;
    /* Driver status */
    volatile uint8_t    state;
    uint32_t frameCount;
     uint32_t 			lastTick;
     float 				fps;

} OV7670;

uint8_t flag_ai_ready = 0;
uint32_t crop_offset = ((240-192)/2 * 320 + (320-192)/2) * 2;
void Crop_Convert_Optimize(const uint8_t *src, int8_t *dst)
{
    // 1. Tính toán vị trí bắt đầu cắt (Căn giữa)
    uint32_t start_x = (SRC_WIDTH - DST_WIDTH) / 2; // (320-192)/2 = 64
    uint32_t start_y = (SRC_HEIGHT - DST_HEIGHT) / 2; // (240-192)/2 = 24

    // Ép kiểu sang uint16_t để đọc 1 lần được cả pixel (2 byte) -> Nhanh hơn đọc từng byte
    const uint16_t *pSrcBase = (const uint16_t *)src;

    // Con trỏ để ghi dữ liệu đầu ra
    int8_t *pDst = dst;

    // 2. Duyệt qua từng dòng của ảnh đích (192 dòng)
    for (uint32_t y = 0; y < DST_HEIGHT; y++)
    {
        // Tính địa chỉ bắt đầu của dòng hiện tại trong ảnh gốc
        // Công thức: (Hàng bắt đầu + Hàng hiện tại) * Độ rộng ảnh gốc + Cột bắt đầu
        const uint16_t *pRowSrc = pSrcBase + ((start_y + y) * SRC_WIDTH) + start_x;

        // 3. Duyệt qua từng điểm ảnh trong dòng (192 điểm)
        for (uint32_t x = 0; x < DST_WIDTH; x++)
        {
            // Đọc 1 pixel (RGB565) từ SDRAM
            uint16_t pixel = *pRowSrc++; // Tự động tăng con trỏ lên pixel tiếp theo

            /* --- CHUYỂN ĐỔI RGB565 -> RGB888 --- */
            // Tách Red (5 bit đầu) -> Đẩy sang 8 bit
            uint8_t r = (pixel >> 11) & 0x1F;
            r = (r * 527 + 23) >> 6; // Công thức chuẩn xác hơn: (r << 3) | (r >> 2)

            // Tách Green (6 bit giữa)
            uint8_t g = (pixel >> 5) & 0x3F;
            g = (g * 259 + 33) >> 6; // Công thức chuẩn: (g << 2) | (g >> 4)

            // Tách Blue (5 bit cuối)
            uint8_t b = pixel & 0x1F;
            b = (b * 527 + 23) >> 6; // Công thức chuẩn: (b << 3) | (b >> 2)

            /* --- GHI VÀO BUFFER AI --- */
            // Lưu ý: Model AI thường cần trừ đi 128 nếu là input int8 (signed)
            // Nếu model dùng uint8 (0-255) thì bỏ "- 128" đi
            *pDst++ = (int8_t)(r - 128);
            *pDst++ = (int8_t)(g - 128);
            *pDst++ = (int8_t)(b - 128);
        }
    }
}
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	if (hdcmi->Instance == OV7670.hdcmi->Instance) {
//		HAL_DCMI_Stop(OV7670.hdcmi);
//		HAL_TIM_OC_Stop(OV7670.htim, OV7670.tim_ch);
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)OV7670.buffer_addr[0], OV7670_FRAME_SIZE_BYTES);
//		Crop_Center_192x192((uint8_t*)OV7670.buffer_addr[0],frame_192x192);
//		RGB565_to_UINT8(frame_192x192, (uint8_t*)data_in_1, 192, 192);
//		Draw_Rectangle_Outline((ILI9341_ACTIVE_WIDTH - 192) / 2, (ILI9341_ACTIVE_HEIGHT - 192) / 2,192,192,BLUE);
		Crop_Convert_Optimize((uint8_t*)OV7670.buffer_addr[0], (int8_t*)data_in_1);
		for(int i=0;i<10;i++){
			sprintf(label, "Ball %.0f%%", ai_score[i] * 100);
	        LCD_PrintString(ai_x[i], ai_y[i] - 12, label);
	        Draw_Rectangle_Outline(ai_x[i]+5,ai_y[i]+10,ai_w[i],ai_h[i],BLUE);

		}
		uint32_t currentTick = HAL_GetTick();
		OV7670.frameCount++;
		 if (currentTick - OV7670.lastTick >= 500) {
			OV7670.fps = (OV7670.frameCount *1000.0 / (currentTick - OV7670.lastTick));
			OV7670.frameCount = 0;
			OV7670.lastTick = currentTick;
		}

		sprintf(label, "FPS: %.2f | X: %.2f | Y: %.2f", OV7670.fps,x_ofs, y_ofs);
		LCD_PrintString((320-192)/2, (240-192)/2 - 12, label);
		ILI9341_DrawFrame((uint8_t*)OV7670.buffer_addr[0],OV7670_FRAME_SIZE_BYTES);
		flag_ai_ready = 1;
		OV7670_START_XLK(OV7670.htim, OV7670.tim_ch);
		HAL_DCMI_Start_DMA(OV7670.hdcmi, DCMI_MODE_SNAPSHOT, OV7670.buffer_addr[0], OV7670_FRAME_SIZE_WORDS);
	}
}

int SDRAM_Test_DataBus(void)
{
    bus_type pattern;
    volatile bus_type *pMem = (bus_type *)SDRAM_BASE_ADDR;
    for (int i = 0; i < (sizeof(bus_type) * 8); i++)
    {
        pattern = (bus_type)(1 << i);
        *pMem = pattern;
        __DSB();

        if (*pMem != pattern) {
            return -1;
        }
    }

    *pMem = 0x00000000; __DSB();
    if (*pMem != 0x00000000) return -2;

    *pMem = 0xFFFFFFFF; __DSB();
    if (*pMem != 0xFFFFFFFF) return -3;

    return 0;
}

int SDRAM_Test_AddressBus(uint32_t size_bytes)
{
    volatile bus_type *base = (bus_type *)SDRAM_BASE_ADDR;
    uint32_t nWords = size_bytes / BUS_WIDTH;
    uint32_t offset;
    uint32_t pattern = 0xAAAAAAAA;
    uint32_t antipattern = 0x55555555;
    for (offset = 1; offset < nWords; offset <<= 1) base[offset] = pattern;
    base[0] = antipattern;
    __DSB();
    for (offset = 1; offset < nWords; offset <<= 1) if (base[offset] != pattern) return offset;
    if (base[0] != antipattern) return -1;
    return 0;
}

int SDRAM_Test_Integrity(void)
{
    volatile bus_type *pMem = (bus_type *)SDRAM_BASE_ADDR;
    uint32_t nWords = SDRAM_SIZE_BYTES / BUS_WIDTH;
    uint32_t i;
    for (i = 0; i < nWords; i++) pMem[i] = (bus_type)i;
    __DSB();
    for (i = 0; i < nWords; i++) if (pMem[i] != (bus_type)i) return i;
    for (i = 0; i < nWords; i++) pMem[i] = ~(bus_type)i;
    __DSB();
    for (i = 0; i < nWords; i++) if (pMem[i] != ~(bus_type)i) return i;
    return 0;
}
void Test_SDRAM(){
	// 1. Test Data Bus
    if (SDRAM_Test_DataBus() != 0) {
          while(1) {
        	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
              HAL_Delay(500); // Lỗi Data
          }
      }

      // 2. Test Address Bus
      if (SDRAM_Test_AddressBus(SDRAM_SIZE_BYTES) != 0) {
          while(1) {
        	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
              HAL_Delay(500); // Lỗi Address
          }
      }
      // 3. Test Full Memory
      if (SDRAM_Test_Integrity() != 0) {
          while(1) {
			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
              HAL_Delay(500); // Lỗi Integrity
          }
      }

      for(int i=0;i<5;i++){
    	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
    	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    	  HAL_Delay(300);
      }
}
void SDRAM_INIT_CHECK(){
	SCB->CCR &= ~SCB_CCR_UNALIGN_TRP_Msk;
	SDRAM_Config.TargetBank      = FMC_SDRAM_CMD_TARGET_BANK1;
	SDRAM_Config.RefreshMode     = W9825G6KH_AUTOREFRESH_MODE_CMD;
	SDRAM_Config.RefreshRate     = REFRESH_COUNT;
	SDRAM_Config.BurstLength     = W9825G6KH_BURST_LENGTH_1;
	SDRAM_Config.BurstType       = W9825G6KH_BURST_TYPE_SEQUENTIAL;
	SDRAM_Config.CASLatency      = W9825G6KH_CAS_LATENCY_3;
	SDRAM_Config.OperationMode   = W9825G6KH_OPERATING_MODE_STANDARD;
	SDRAM_Config.WriteBurstMode  = W9825G6KH_WRITEBURST_MODE_SINGLE;
	ret = W9825G6KH_Init(&hsdram1, &SDRAM_Config);
	if(ret != W9825G6KH_OK)
	{
		Error_Handler();
	}
//	Test_SDRAM();
}
uint8_t I2C2_Scan_To_Array(uint8_t *store_arr, uint8_t limit)
{
    uint8_t i;
    uint8_t count = 0;
    HAL_StatusTypeDef result;

    // 1. Xóa sạch mảng trước khi quét (để tránh dữ liệu rác cũ)
    for (i = 0; i < limit; i++) {
        store_arr[i] = 0;
    }

    // 2. Quét từ địa chỉ 1 đến 127
    for (i = 1; i < 128; i++)
    {
        // Kiểm tra nếu mảng đã đầy thì dừng lại
        if (count >= limit) {
            break;
        }

        // HAL_I2C_IsDeviceReady yêu cầu địa chỉ dịch trái 1 bit
        result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 1, 10);

        if (result == HAL_OK)
        {
            // Lưu địa chỉ 7-bit (Ví dụ 0x68 cho MPU6050) vào mảng
            store_arr[count] = i;
            count++;
        }
    }

    return count; // Trả về số lượng tìm thấy
}
uint16_t AngleToPWM(uint8_t angle) {
    if (angle > 180) angle = 180;
    // Sử dụng tính toán số thực để giữ độ chính xác trước khi ép kiểu
    float pulse = (float)SERVOMIN + ((float)angle * (SERVOMAX - SERVOMIN) / 180.0f);
    return (uint16_t)pulse;
}
void SetServoAngle_1(uint8_t num, float angle) {
    uint16_t pwmValue = AngleToPWM(angle);
    PCA9685_SetPWM(&hi2c2,PCA9685_I2C_ADDRESS_1,num, 0, pwmValue);
}
void SetServoAngle_2(uint8_t num, float angle) {
    uint16_t pwmValue = AngleToPWM(angle);
    PCA9685_SetPWM(&hi2c2,PCA9685_I2C_ADDRESS_2,num, 0, pwmValue);
}


void Read_IMU(){
	d = bno055_getVectorEuler();
	goc_ht[0] = d.x;
	delta_goc[0] = goc_ht[0] - goc_trc[0];
	if (delta_goc[0] > 180) delta_goc[0] -= 360;
	if (delta_goc[0] < -180) delta_goc[0] += 360;
	goc_tong[0] += delta_goc[0];
	goc_trc[0] = goc_ht[0];



	goc_ht[1] = d.z;
	delta_goc[1] = goc_ht[1] - goc_trc[1];
	if (delta_goc[1] > 180) delta_goc[1] -= 360;
	if (delta_goc[1] < -180) delta_goc[1] += 360;
	goc_tong[1] += delta_goc[1];
	goc_trc[1] = goc_ht[1];
}
float my_clamp(float fl, float min, float max){
	if(fl<min)return min;
	if(fl>max)return max;
	return fl;
}
bool solve_ik(double dx, double dy, double dz, double *a, bool is_r) {
        // Tối ưu hóa: Nếu chân bị duỗi quá dài (vượt giới hạn vật lý), thu hẹp Y để cứu Z
        double d_target = sqrt(dx*dx + dy*dy + dz*dz);
        double MAX_REACH = L3 + L4 + L5 - 0.002; // Chừa 2mm an toàn cho gối

        if (d_target > MAX_REACH) {
            double scale = MAX_REACH / d_target;
            dx *= scale; dy *= scale; dz *= scale;
        }

        double hn = atan2(dy, dz);
        double d_yz = sqrt(dz * dz + dy * dy);
        double c3_raw = (pow(d_yz - L5, 2) + dx * dx - L3 * L3 - L4 * L4) / (2.0 * L3 * L4);

        // Clamp lỏng hơn một chút (-1.05) để tránh mất giải nghiệm khi chân duỗi thẳng ở Z=0.195
        if (c3_raw < -1.05 || c3_raw > 1.05) return false;

        double c3 = my_clamp(c3_raw, -1.0, 1.0);

        double s3 = sqrt(1.0 - c3 * c3);
        double dg = atan2(s3, c3); // Joint Knee
        double ht = atan2(s3 * L4, L3 + c3 * L4) + atan2(dx, d_yz - L5); // Joint Hip pitch

        a[0] = hn*rad2deg;             // Hip roll
        a[1] = (is_r ? ht : -ht)*rad2deg; // Hip pitch (đảo dấu cho chân trái)
        a[2] = (is_r ? -dg : dg)*rad2deg; // Knee (đảo dấu cho chân phải)
        a[3] = (ht - dg)*rad2deg; // Ankle pitch
        a[4] = -hn*rad2deg;            // Ankle roll
        return true;
    }

void Update_Observation_Buffer(void) {
    // 1. Cập nhật pha CPG (dt=0.05, T=1.0)
    cpg_phase += 2.0f * M_PI * 0.05f / 1.0f; 
    if (cpg_phase >= 2.0f * M_PI) {
        cpg_phase -= 2.0f * M_PI;
    }
    
    // 2. Lấy dữ liệu cảm biến BNO055
    bno055_vector_t q = bno055_getVectorQuaternion();     // w, x, y, z
    bno055_vector_t gyro = bno055_getVectorGyroscope();   // rad/s
    bno055_vector_t lin = bno055_getVectorLinearAccel();  // m/s^2
    
    float current_frame[18];
    
    // [0-3] Quaternion chuẩn hóa
    current_frame[0] = (float)q.w;
    current_frame[1] = (float)q.x;
    current_frame[2] = (float)q.y;
    current_frame[3] = (float)q.z;
    
    // [4-6] Vận tốc góc / MAX_GYRO=10.0 -> [-1, 1]
    current_frame[4] = my_clamp((float)gyro.x, -10.0f, 10.0f) / 10.0f;
    current_frame[5] = my_clamp((float)gyro.y, -10.0f, 10.0f) / 10.0f;
    current_frame[6] = my_clamp((float)gyro.z, -10.0f, 10.0f) / 10.0f;
    
    // [7-9] Gia tốc tuyến tính / 20.0 -> [-1, 1]
    current_frame[7] = my_clamp((float)lin.x, -20.0f, 20.0f) / 20.0f;
    current_frame[8] = my_clamp((float)lin.y, -20.0f, 20.0f) / 20.0f;
    current_frame[9] = my_clamp((float)lin.z, -20.0f, 20.0f) / 20.0f;
    
    // [10-15] Hành động thô bước trước 
    for(int i = 0; i < 6; i++) {
        current_frame[10+i] = prev_raw_action[i];
    }
    
    // [16-17] Pha CPG
    current_frame[16] = sinf(cpg_phase);
    current_frame[17] = cosf(cpg_phase);
    
    // 3. Thay đổi buffer lịch sử (Dịch chuyển cửa sổ về quá khứ)
    for (int i = 0; i < 6 * 18; i++) {
        history_buffer[i] = history_buffer[i + 18];
    }
    
    // 4. Push frame mới nhất vào
    for (int i = 0; i < 18; i++) {
        history_buffer[108 + i] = current_frame[i];
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FMC_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_DCMI_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_DMA2D_Init();
  MX_ADC3_Init();
  MX_USB_DEVICE_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
//  SDRAM_INIT_CHECK();
  HAL_Delay(100);
//  MX_X_CUBE_AI_Init();
//  ILI9341_Init(&hspi2, ILI9341_PIXEL_FMT_RGB565);
//  ILI9341_DrawFrame(LOGO, LOGO_size);
//  dev_count = I2C2_Scan_To_Array(my_i2c_devices, 10);
  PCA9685_Init(&hi2c2, PCA9685_I2C_ADDRESS_1);
  PCA9685_SetPWMFreq(&hi2c2,PCA9685_I2C_ADDRESS_1, 50.0);
  PCA9685_Init(&hi2c2, PCA9685_I2C_ADDRESS_2);
  PCA9685_SetPWMFreq(&hi2c2,PCA9685_I2C_ADDRESS_2, 50.0);

//  OV7670_Init(&hdcmi, &hi2c1, &htim5, TIM_CHANNEL_3);
//  HAL_Delay(100);
//  OV7670_Start();
	bno055_assignI2C(&hi2c3);
	bno055_setup();
	bno055_setOperationModeNDOF();
	SetServoAngle_1(9, 95);//than
	SetServoAngle_2(9, 105);
	if (solve_ik(0, 0.01, 0.265, l_a, false) &&
			solve_ik(0, -0.01, 0.265, r_a, true)) {

		SetServoAngle_1(0, 90 + l_a[4]);
		SetServoAngle_1(1, 90 - (-15+l_a[3]));
		SetServoAngle_1(2, 90 + l_a[2]);
		SetServoAngle_1(3, 90 - l_a[1]);
		SetServoAngle_1(4, 55 + l_a[0]);
		SetServoAngle_1(5, 80);
		SetServoAngle_1(6, 92);
		SetServoAngle_1(7, 98);
		SetServoAngle_1(8, 60);


		SetServoAngle_2(0, 90-r_a[4]);
		SetServoAngle_2(1, 85+(-15+r_a[3]));
		SetServoAngle_2(2, 90+r_a[2]);
		SetServoAngle_2(3, 90-r_a[1]);
		SetServoAngle_2(4, 70+r_a[0]);
		SetServoAngle_2(5, 80);
		SetServoAngle_2(6, 86);
		SetServoAngle_2(7, 86);
		SetServoAngle_2(8, 65);

		HAL_Delay(100);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  last_inference_time = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */

  // MX_X_CUBE_AI_Process(); // Default location (disabled to control frequency)

    /* USER CODE BEGIN 3 */
    // --- 20Hz TIME-STEP (50ms) ---
    if (HAL_GetTick() - last_inference_time >= 50) {
        last_inference_time = HAL_GetTick();
        
        // A -> CPG step & B -> build_frame
        Update_Observation_Buffer();
        
        // C -> Running AI (Calls acquire -> run -> post_process)
        MX_X_CUBE_AI_Process(); 
        
        // D -> Pipeline CPG + Residual + EMA + IK
        for(int i = 0; i < 6; i++) {
            // Update raw_prev for the NEXT build_frame
            prev_raw_action[i] = rl_action[i];
            
            // EMA smoothing
            float res = rl_action[i] * scale_array[i];
            smoothed_action[i] = EMA_ALPHA * res + (1.0f - EMA_ALPHA) * smoothed_action[i];
        }

        float tx_l = smoothed_action[0];
        float tx_r = smoothed_action[1];
        float target_y_l = STD_Y + smoothed_action[2];
        float target_y_r = -STD_Y + smoothed_action[2];
        float tz = STD_Z + smoothed_action[3]; // Not used in Phase 1 as dz=0, but kept for scalability

        // Apply Lift
        float tz_l = tz - smoothed_action[4];
        float tz_r = tz - smoothed_action[5];

        // Knee protection / IK Clamp
        if (tz_l < 0.12f) tz_l = 0.12f; 
        if (tz_l > 0.285f) tz_l = 0.285f;
        
        if (tz_r < 0.12f) tz_r = 0.12f; 
        if (tz_r > 0.285f) tz_r = 0.285f;

        // E -> Publish to Servos
        if (solve_ik(tx_l, target_y_l, tz_l, l_a, false) &&
            solve_ik(tx_r, target_y_r, tz_r, r_a, true)) {
            
            // Servo Action
            SetServoAngle_1(0, 90 + l_a[4]);
            SetServoAngle_1(1, 90 - (-15 + l_a[3]));
            SetServoAngle_1(2, 90 + l_a[2]);
            SetServoAngle_1(3, 90 - l_a[1]);
            SetServoAngle_1(4, 55 + l_a[0]);
            SetServoAngle_1(5, 80); SetServoAngle_1(6, 92); SetServoAngle_1(7, 98); SetServoAngle_1(8, 60);

            SetServoAngle_2(0, 90 - r_a[4]);
            SetServoAngle_2(1, 85 + (-15 + r_a[3]));
            SetServoAngle_2(2, 90 + r_a[2]);
            SetServoAngle_2(3, 90 - r_a[1]);
            SetServoAngle_2(4, 70 + r_a[0]);
            SetServoAngle_2(5, 80); SetServoAngle_2(6, 86); SetServoAngle_2(7, 86); SetServoAngle_2(8, 65);
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 19;
  PeriphClkInitStruct.PLL3.PLL3P = 1;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x307075B1;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2399;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 2399;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 99;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 12;
  SdramTiming.SelfRefreshTime = 7;
  SdramTiming.RowCycleDelay = 10;
  SdramTiming.WriteRecoveryTime = 4;
  SdramTiming.RPDelay = 3;
  SdramTiming.RCDDelay = 3;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESX_GPIO_Port, LCD_RESX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, LCD_DCX_Pin|LCD_CSX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CAM_PWDN_Pin|CAM_RET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin PC4 */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESX_Pin */
  GPIO_InitStruct.Pin = LCD_RESX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_RESX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DCX_Pin */
  GPIO_InitStruct.Pin = LCD_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DCX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CSX_Pin */
  GPIO_InitStruct.Pin = LCD_CSX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_CSX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_PWDN_Pin CAM_RET_Pin */
  GPIO_InitStruct.Pin = CAM_PWDN_Pin|CAM_RET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER15;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RW;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
