/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      main.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */

/* Includes ------------------------------------------------------------------*/

#include <dmp_mpu6050.h>

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)

#define I2C_TIMEOUT 1000

struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned char motion_int_mode;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned long next_temp_ms;
	unsigned long next_compass_ms;
	unsigned int report;
	unsigned short dmp_features;

};
static struct hal_s hal = { 0 };

unsigned char *mpl_key = (unsigned char*) "eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
	signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = { .orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 } };

/**
 * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��д��ĳ���
 *	@param data_ptr:ָ��Ҫд�������
 * @retval ����Ϊ0��������Ϊ��0
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
		unsigned char *data_ptr) {

	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1, slave_addr, reg_addr,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) data_ptr, len, I2C_TIMEOUT);

	/* ���ͨѶ״̬ */
	if (status != HAL_OK) {
		/* ���߳����� */
		printf("I2C Write Wrong!!\r\n");
		return HAL_ERROR;
	}

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}

	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr, 50, I2C_TIMEOUT) == HAL_TIMEOUT)
		;

	/* �ȴ�������� */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {

	}

	return status;
}

/**
 * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��Ҫ��ȡ�ĳ���
 *	@param data_ptr:ָ��Ҫ�洢���ݵ�ָ��
 * @retval ����Ϊ0��������Ϊ��0
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
		unsigned char *data_ptr) {

//	printf("%d %d\n", reg_addr, len);

	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, slave_addr, reg_addr,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) data_ptr, len, I2C_TIMEOUT);

	/* ���ͨѶ״̬ */
	if (status != HAL_OK) {
		/* ���߳����� */
		printf("I2C Read Wrong!!\r\n");
		return HAL_ERROR;
	}

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}

	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr, 50, I2C_TIMEOUT) == HAL_TIMEOUT)
		;

	/* �ȴ�������� */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {

	}

	return status;
}

/**
 * @brief  ��ȡ��ǰ����ֵ
 * @param  �洢���º���ֵ�ı���
 * @retval ��
 */
int get_tick_count(unsigned long *count) {
	count[0] = HAL_GetTick();
	return 0;
}

int fputcc(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 1000);
	return ch;
}

static inline int run_self_test(void) {

	int result;
	long gyro[3], accel[3];
	result = mpu_run_self_test(gyro, accel);

	if (result != 0x7) {
		if (!(result & 0x1))
			printf("Gyro failed.\n");
		if (!(result & 0x2))
			printf("Accel failed.\n");
		if (!(result & 0x4))
			printf("Compass failed.\n");
		return -1;
	}

	printf("Passed!\n");
	printf("accel: %.2f %.2f %.2f\n", accel[0] / 65536.f, accel[1] / 65536.f, accel[2] / 65536.f);
	printf("gyro: %.2f %.2f %.2f\n", gyro[0] / 65536.f, gyro[1] / 65536.f, gyro[2] / 65536.f);
	/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

	/* Push the calibrated data to the MPL library.
	 *
	 * MPL expects biases in hardware units << 16, but self test returns
	 * biases in g's << 16.
	 */
	unsigned short accel_sens;
	float gyro_sens;

	mpu_get_accel_sens(&accel_sens);
	accel[0] *= accel_sens;
	accel[1] *= accel_sens;
	accel[2] *= accel_sens;
	inv_set_accel_bias(accel, 3);

	mpu_get_gyro_sens(&gyro_sens);
	gyro[0] = (long) (gyro[0] * gyro_sens);
	gyro[1] = (long) (gyro[1] * gyro_sens);
	gyro[2] = (long) (gyro[2] * gyro_sens);
	inv_set_gyro_bias(gyro, 3);

	return 0;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void) {
	hal.new_gyro = 1;
}

uint32_t lsti, cnti, lstci;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_9) {
		cnti++;
		if (HAL_GetTick() - lsti >= 1000) {
			lstci = cnti;
			cnti = 0;
			lsti = HAL_GetTick();
		}
		gyro_data_ready_cb();
	}
}

/*******************************************************************************/

/**
 * @brief main entry point.
 * @par Parameters None
 * @retval void None
 * @par Required preconditions: None
 */

int MPU6050_Init(void) {

//	inv_error_t result;
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	unsigned long timestamp;
	struct int_param_s int_param;

	if (mpu_init(&int_param)) {
		printf("Could not initialize gyro.\n");
		return -1;
	} else {
		printf("Gyro initialized.\n");
	}

	/* If you're not using an MPU9150 AND you're not using DMP features, this
	 * function will place all slaves on the primary bus.
	 * mpu_set_bypass(1);
	 */

	if (inv_init_mpl()) {
		printf("Could not initialize MPL.\n");
		return -1;
	} else {
		printf("MPL initialized.\n");
	}

	/* Compute 6-axis and 9-axis quaternions. */
	inv_enable_quaternion();
	inv_enable_9x_sensor_fusion();

	/* The MPL expects compass data at a constant rate (matching the rate
	 * passed to inv_set_compass_sample_rate). If this is an issue for your
	 * application, call this function, and the MPL will depend on the
	 * timestamps passed to inv_build_compass instead.
	 *
	 * inv_9x_fusion_use_timestamps(1);
	 */

	/* This function has been deprecated.
	 * inv_enable_no_gyro_fusion();
	 */

	/* Update gyro biases when not in motion.
	 * WARNING: These algorithms are mutually exclusive.
	 */
	inv_enable_fast_nomot();
//	inv_enable_motion_no_motion();
	inv_set_no_motion_time(2000);

	/* Update gyro biases when temperature changes. */
	inv_enable_gyro_tc();

	/* This algorithm updates the accel biases when in motion. A more accurate
	 * bias measurement can be made when running the self-test (see case 't' in
	 * handle_input), but this algorithm can be enabled if the self-test can't
	 * be executed in your application.
	 *
	 * inv_enable_in_use_auto_calibration();
	 */

	/* If you need to estimate your heading before the compass is calibrated,
	 * enable this algorithm. It becomes useless after a good figure-eight is
	 * detected, so we'll just leave it out to save memory.
	 * inv_enable_heading_from_gyro();
	 */

	/* Allows use of the MPL APIs in read_from_mpl. */
	inv_enable_eMPL_outputs();

	if (inv_start_mpl()) {
		printf("Could not start the MPL.\n");
		return -1;
	}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
	/* Push both gyro and accel data into the FIFO. */
	/* Read back configuration in case it was set improperly. */

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)
			|| mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)
			|| mpu_set_sample_rate(DEFAULT_MPU_HZ) || mpu_get_sample_rate(&gyro_rate)
			|| mpu_get_gyro_fsr(&gyro_fsr) || mpu_get_accel_fsr(&accel_fsr)) {
		return -1;
	}

	/* Sync driver configuration with MPL. */
	/* Sample rate expected in microseconds. */
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);

	/* Set chip-to-body orientation matrix.
	 * Set hardware units to dps/g's/degrees scaling factor.
	 */
	inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long) gyro_fsr << 15);
	inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long) accel_fsr << 15);

	/* Initialize HAL state variables. */
	hal.sensors = ACCEL_ON | GYRO_ON;
	hal.dmp_on = 0;
	hal.report = 0;
	hal.next_pedo_ms = 0;
	hal.next_temp_ms = 0;

	/* Compass reads are handled by scheduler. */
	get_tick_count(&timestamp);

	/* To initialize the DMP:
	 * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
	 *    inv_mpu_dmp_motion_driver.h into the MPU memory.
	 * 2. Push the gyro and accel orientation matrix to the DMP.
	 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
	 *    executed unless the corresponding feature is enabled.
	 * 4. Call dmp_enable_feature(mask) to enable different features.
	 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	 * 6. Call any feature-specific control functions.
	 *
	 * To enable the DMP, just call mpu_set_dmp_state(1). This function can
	 * be called repeatedly to enable and disable the DMP at runtime.
	 *
	 * The following is a short summary of the features supported in the DMP
	 * image provided in inv_mpu_dmp_motion_driver.c:
	 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	 * 200Hz. Integrating the gyro data at higher rates reduces numerical
	 * errors (compared to integration on the MCU at a lower sampling rate).
	 * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
	 * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
	 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	 * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
	 * an event at the four orientations where the screen should rotate.
	 * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
	 * no motion.
	 * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
	 * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
	 * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
	 * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
	 */

	if (dmp_load_motion_driver_firmware()
			|| dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation))) {
		return -1;
	}

	/*
	 * Known Bug -
	 * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
	 * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
	 * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
	 * there will be a 25Hz interrupt from the MPU device.
	 *
	 * There is a known issue in which if you do not enable DMP_FEATURE_TAP
	 * then the interrupts will be at 200Hz even if fifo rate
	 * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
	 *
	 * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
	 */
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT
			| DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;

	if (dmp_enable_feature(hal.dmp_features) || dmp_set_fifo_rate(DEFAULT_MPU_HZ)
			|| mpu_set_dmp_state(1)) {
		return -1;
	}

	hal.dmp_on = 1;

	if (run_self_test()) {
		return -1;
	}

	inv_accel_was_turned_off();
	inv_gyro_was_turned_off();

	return 0;
}

int MPU6050_Go() {

	if(!hal.new_gyro){
		return 0;
	}

	unsigned long sensor_timestamp;
	int new_data = 0;

	unsigned long timestamp;
	char new_temp = 0;

	get_tick_count(&timestamp);

	/* Temperature data doesn't need to be read with every gyro sample.
	 * Let's make them timer-based like the compass reads.
	 */
	if (timestamp > hal.next_temp_ms) {
		hal.next_temp_ms = timestamp + TEMP_READ_MS;
		new_temp = 1;
	}

	if (hal.dmp_on) {

		short gyro[3], accel_short[3], sensors;
		unsigned char more;
		long accel[3], quat[4], temperature;
		/* This function gets new data from the FIFO when the DMP is in
		 * use. The FIFO can contain any combination of gyro, accel,
		 * quaternion, and gesture data. The sensors parameter tells the
		 * caller which data fields were actually populated with new data.
		 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
		 * the FIFO isn't being filled with accel data.
		 * The driver parses the gesture data to determine if a gesture
		 * event has occurred; on an event, the application will be notified
		 * via a callback (assuming that a callback function was properly
		 * registered). The more parameter is non-zero if there are
		 * leftover packets in the FIFO.
		 */
		if (dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more)) {
			return -1;
		}

		if (!more)
			hal.new_gyro = 0;

		if (sensors & INV_XYZ_GYRO) {
			/* Push the new data to the MPL. */
			inv_build_gyro(gyro, sensor_timestamp);
			new_data = 1;
			if (new_temp) {
				new_temp = 0;
				/* Temperature only used for gyro temp comp. */
				if (mpu_get_temperature(&temperature, &sensor_timestamp)) {
					return -1;
				}
				inv_build_temp(temperature, sensor_timestamp);
			}
		}
		if (sensors & INV_XYZ_ACCEL) {
			accel[0] = (long) accel_short[0];
			accel[1] = (long) accel_short[1];
			accel[2] = (long) accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
		}
		if (sensors & INV_WXYZ_QUAT) {
			inv_build_quat(quat, 0, sensor_timestamp);
			new_data = 1;
		}
	}

	if (new_data) {
		inv_execute_on_data();
		return 1;
	}

	return 0;
}

