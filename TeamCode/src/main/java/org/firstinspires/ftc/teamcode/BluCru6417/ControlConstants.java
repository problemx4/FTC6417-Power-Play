package org.firstinspires.ftc.teamcode.BluCru6417;

public interface ControlConstants {
    /* measurement variables */
    // calculate these in inches
    double DIAMETER      = 3.77952756;
    double CPR           = 800;
    double CIRC          = DIAMETER * Math.PI;

    //angle variables
    int armVerticalPosition         = 100; //MEASURE THIS
    double wristVerticalPosition    = .5; //MEASURE THIS
    double wristHorizontalPosition  = 1.0 //MEASURE THIS

    //maintenance variable
    double armMaintenanceCoeff      = .01; //MEASURE THIS


    /* control variables */
    //power variables
    double maxDrivePower        = 1.0;
    double maxSlowerDrivePower  = 0.3;
    double maxSliderPower       = 0.4;
    double maxArmPower          = 0.2;

    double sens                 = 0.05;
    double triggerSens          = 0.5;

    double grabberClosePos      = 0.0;
    double grabberOpenPos       = 0.12;
    double manualWristDelta     = 0.01;

    //encoder tick variables
    int sliderBasePos           = 0;
    int sliderLowPos            = 100;
    int sliderMedPos            = 200;
    int sliderHighPos           = 300;
    int sliderMaxPos            = 400;

    int armDownPos              = 0;
    int armForwardPos           = armVerticalPosition;
    int armBackwardsPos         = -armVerticalPosition;
    int armUsefulPos            = 100;
}
