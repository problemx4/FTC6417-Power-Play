package org.firstinspires.ftc.teamcode.BluCru6417;

public interface ControlConstants {
    /* measurement variables */
    // calculate these in inches
    double DIAMETER                 = 3.77952756;
    double CPR                      = 800;
    double CIRC                     = DIAMETER * Math.PI;

    //angle variables
    int armHorizontalPosition       = 100; //MEASURE THIS

    double armServoVerticalPosition = .5; //MEASURE THIS
    double armServoHorizontalPosition = 1.0; //MEASURE THIS

    double wristVerticalPosition    = .5; //MEASURE THIS
    double wristHorizontalPosition  = 1.0;

    //maintenance variable
    double armMaintenanceCoeff      = .01; //MEASURE THIS


    /* control variables */
    //power variables
    double maxDrivePower            = 0.6;
    double maxSlowerDrivePower      = 0.3;
    double maxSliderPower           = 0.4;
    double maxArmPower              = 0.2;

    double autoSlideSpeed           = 0.6;
    double autoArmSpeed             = 0.5;

    double sens                     = 0.05;
    double triggerSens              = 0.5;

    double manualWristDelta         = 0.01;

    double grabberClosePos          = 0.0;
    double grabberOpenPos           = 0.12;

    double armServoDownPos          = 0.5;
    double armServoForwardPos       = armServoHorizontalPosition;
    double armServoBackwardsPos     = -armServoHorizontalPosition;
    double armServoUsefulPos        = 0.75;

    //encoder tick variables
    int sliderBasePos               = 0;
    int sliderLowPos                = 100;
    int sliderMedPos                = 200;
    int sliderHighPos               = 300;
    int sliderMaxPos                = 400;

    int armDownPos                  = 0;
    int armForwardPos               = armHorizontalPosition;
    int armBackwardsPos             = -armHorizontalPosition;
    int armUsefulPos                = 100;
}
