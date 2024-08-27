
/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include "usart.h"
#include "i2c.h"
#include "gpio.h"
#include "main.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void gyro_data_ready_cb(void);

int get_tick_count(unsigned long *count);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len,unsigned char *data_ptr);
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr);
																				
int MPU6050_Init(void);
int fputcc(int ch);
int MPU6050_Go();
