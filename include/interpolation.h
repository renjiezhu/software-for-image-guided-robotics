#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <vector>
#include <string>

// Reflexxes library for interpolation
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

#define CYCLE_TIME_IN_SECONDS                   0.001
#define NUMBER_OF_DOFS                          8


class interpolation{

public:

    std::vector<double>              setpoint;
    
    interpolation() {
        // Variable declarations and definitions
        ResultValue                 =   0       ;
        RML                         =   NULL    ;
        IP                          =   NULL    ;
        OP                          =   NULL    ;

/*
        double targetPosition[NUMBER_OF_DOFS] = {}; //should be 8 for our application
        double targetVelocity[NUMBER_OF_DOFS] = {}; //should be 8 for our application
        double currentPosition[NUMBER_OF_DOFS] = {};
        double currentVelocity[NUMBER_OF_DOFS] = {};
        double currentAcceleration[NUMBER_OF_DOFS] = {};
*/
        


        // Creating all relevant objects of the Type II Reflexxes Motion Library
        RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                                ,   CYCLE_TIME_IN_SECONDS   );
        IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );
        OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

        // Time-synchronization
        //Flags.SynchronizationBehavior   =   RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

        std::string base_path = "/home/arclab-rjz/Documents/igr/src/software_interface/include/limit/"
        std::vector<std::string> motorAddress(8);
        for (int i=0; i<8; ++i) {
            motorAddress[i] = base_path + "motor" + std::to_string(i) + ".json"
        }

        // Set up limit for maximum velocity, acceleration, and jerk;
        for (int i=0; i<NUMBER_OF_DOFS; i++){
            std::ifstream motor_config_file(motorAddress[i]);
            json motor_config = json::parse(motor_config_file);
            IP->MaxVelocityVector->VecData[i] = motor_config["joint_limits"]["velocity_limit"];
            IP->MaxAccelerationVector->VecData[i] = motor_config["joint_limits"]["torque_limit"];
            IP->MaxJerkVector->VecData[i] = motor_config["joint_limits"]["jerk_limit"];
            IP->SelectionVector->VecData[i] = true;
        }

        // Add JSON parser later-----------------------------------------

        printf("-------------------------------------------------------\n"  );
        printf("Reflexxes Motion Libraries activated!                  \n"  );
        printf("Performing interpolation on trajectories...            \n"  );
    }

    ~interpolation() {
        // Delete objects when leaving the scope
        delete RML;
        delete IP;
        delete OP;
    }

    // Update set point
    void updateSetPoint(std::vector<double> position, std::vector<double> velocity);


protected:

    bool                        IntermediateTargetStateSet;
    bool                        IntermediateStateReached;
    int                         ResultValue;
    double                      Time;
    ReflexxesAPI                *RML;
    RMLPositionInputParameters  *IP;
    RMLPositionOutputParameters *OP;
    RMLPositionFlags            Flags;

    double targetPosition[NUMBER_OF_DOFS];
    double targetVelocity[NUMBER_OF_DOFS];
    double currentPosition[NUMBER_OF_DOFS];
    double currentVelocity[NUMBER_OF_DOFS];
    double currentAcceleration[NUMBER_OF_DOFS];
    

};