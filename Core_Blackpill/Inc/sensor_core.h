/*
 * sensor_core.h
 *
 *  Created on: Mar 5, 2025
 *      Author: danba
 */

#ifndef INC_SENSOR_CORE_H_
#define INC_SENSOR_CORE_H_

#define WATER_THRESHOLD  0


#define safe_threshold 0
#define caution_threshold 0
#define dangerous_threshold 0


int water_detection(void);
void water_turbidity(void);
void mq7_gas_sensor(void);
void mq2_gas_sensor(void);
void mq8_gas_sensor(void);



#endif /* INC_SENSOR_CORE_H_ */
