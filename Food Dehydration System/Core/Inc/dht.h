/*
 * DHT.h
 *
 *  Created on: Jun 28, 2020
 *      Author: Controllerstech.com
 */

#ifndef DHT_H_
#define DHT_H_



typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;


void DHT_GetData (DHT_DataTypedef *DHT_Data, GPIO_TypeDef* PORT, uint16_t PIN  );

#endif /* INC_DHT_H_ */
