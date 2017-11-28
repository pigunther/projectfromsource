/**
 * @file    VL53L0X.h
 * @brief   High level functions to use the VL53L0X TOF sensor.
 * 
 * @author  Eliot Ferragni
 */


#ifndef VL53L0X_H
#define VL53L0X_H

#include "Api/core/inc/vl53l0x_api.h"

#define USE_I2C_2V8

#define VL53L0X_ADDR 0x52

//////////////////// PROTOTYPES PUBLIC FUNCTIONS /////////////////////

/**
 * @brief 			Init the VL53L0X_Dev_t structure and the sensor.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */	
VL53L0X_Error VL53L0X_init(VL53L0X_Dev_t* device);

/**
 * @brief 			Configure the accuracy of the sesor (range).
 * 
 * @param device 	Pointer to the structure of the sensor
 * @param accuracy 	Accuracy chosen
 * 
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_configAccuracy(VL53L0X_Dev_t* device, VL53L0X_AccuracyMode accuracy);

/**
 * @brief 			Begin the meausurement process with the specified mode.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @param mode 		Mode chosen to take the measures
 * 
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_startMeasure(VL53L0X_Dev_t* device, VL53L0X_DeviceModes mode);

/**
 * @brief 			Get the last valid measure and lpace it in the sensor structure given.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_getLastMeasure(VL53L0X_Dev_t* device);

/**
 * @brief 			Stop the measuement process.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */	
VL53L0X_Error VL53L0X_stopMeasure(VL53L0X_Dev_t* device);

/**
 * @brief Init a demo thread which uses the three proximity sensors to
 * continuoulsy measure the distance and varies the intensity of the leds to 
 * represent the measured distances.
 */
void VL53L0X_init_demo(void);

#endif /* VL53L0X_H*/