#include "interpolation.h"

#include <chrono>
#include <vector>
#include <algorithm>

void interpolation::updateSetPoint(std::vector<double> position, std::vector<double> velocity){


    std::copy(position.begin(), position.end(), targetPosition);
    std::copy(velocity.begin(), velocity.end(), targetVelocity);

    IP->TargetPositionVector->VecData = targetPosition;
    IP->TargetVelocityVector->VecData = targetVelocity;

    ResultValue =   RML->RMLPosition(       *IP
                                        ,   OP
                                        ,   Flags       );

    if (ResultValue < 0)
    {
        printf("An error occurred (%d).\n", ResultValue );
    }

    std::vector<double> setpoint(NUMBER_OF_DOFS);
    for (int i=0; i<NUMBER_OF_DOFS; ++i) {
        setpoint[i] = OP->NewPositionVector->VecData[i];
    }
    this->setpoint = setpoint;

    /*
    std::vector<double> encoder_value= mManager.getFPGAEncoderPositions();
    for (int i=0; i<NUMBER_OF_DOFS; ++i) {
        IP->CurrentPositionVector->VecData[i]      =   encoder_value[i];
    }
    */

    *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
    *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
    *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

}



