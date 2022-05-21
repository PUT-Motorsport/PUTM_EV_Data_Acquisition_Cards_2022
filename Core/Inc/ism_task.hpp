/*
 * ism_task.hpp
 *
 *  Created on: May 21, 2022
 *      Author: molso
 */

#ifndef INC_ISM_TASK_HPP_
#define INC_ISM_TASK_HPP_

#define SENSOR_ODR 52.0f // In Hertz
#define ACC_FS 2 // In g
#define GYR_FS 1000 // In dps
#define MEASUREMENT_TIME_INTERVAL (1000.0f/SENSOR_ODR) // In ms
#define FIFO_SAMPLE_THRESHOLD 199
#define FLASH_BUFF_LEN 8192
class ism_task {
public:
	void ism330_setup(void);
	void ism330_setup(void);
};

#endif /* INC_ISM_TASK_HPP_ */
