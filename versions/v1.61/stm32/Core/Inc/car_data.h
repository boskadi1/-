#ifndef CAR_DATA_H_
#define CAR_DATA_H_

#include "car_types.h"

extern CarData_t gCar;

void CarData_Init(void);
void CarData_ResetMission(CarMode_t mode);
const char *CarData_ActionName(CarAction_t action);
const char *CarData_ModeName(CarMode_t mode);
const char *CarData_StageName(CarStage_t stage);

#endif /* CAR_DATA_H_ */
