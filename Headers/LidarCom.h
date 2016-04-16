/****************************************************************************
 Module
     LidarCom.h
 Description
     Header file for interface with the Lidar Module over I2C
 ****************************************************************************/
#ifndef LIDAR_COM_H
#define LIDAR_COM_H
/*----------------------------- Include Files -----------------------------*/

/*----------------------------- Module Defines ----------------------------*/
typedef struct {
	uint8_t distanceHIGH;
	uint8_t distanceLOW;
} LidarData_t;

/*----------------------- Public Function Prototypes ----------------------*/
bool InitLidarCom (uint8_t Priority);
ES_Event RunLidarCom(ES_Event ThisEvent);
bool PostLidarCom(ES_Event ThisEvent);

LidarData_t getLidarData(void);
#endif /* LIDAR_COM_H */
