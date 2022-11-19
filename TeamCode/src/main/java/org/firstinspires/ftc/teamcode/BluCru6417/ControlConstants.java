package org.firstinspires.ftc.teamcode.BluCru6417;

public interface ControlConstants {
    //angle variables
    double armServoVerticalPosition = .5; //MEASURE THIS
    double armServoHorizontalPosition = 1.0; //MEASURE THIS

    double wristVerticalPosition    = .5; //MEASURE THIS
    double wristHorizontalPosition  = 1.0;


    /* control variables */
    //power variables
    double maxDrivePower            = 0.6;
    double maxSlowerDrivePower      = 0.3;

    double maxSliderPower           = 0.4;
    double returnSliderPower        = 0.15;
    double autoSlidePower           = 0.6;

    double sens                     = 0.05;
    double triggerSens              = 0.5;

    double manualWristDelta         = 0.01;

    double grabberClosePos          = 1.0;
    double grabberOpenPos           = 0.78;

    double armServoDownPos          = 0.5;
    double armServoForwardPos       = armServoHorizontalPosition;
    double armServoBackwardsPos     = -armServoHorizontalPosition;
    double armServoUsefulPos        = 0.75;

    //encoder tick variables
    int sliderBasePos               = 0;
    int sliderLowPos                = 1000;
    int sliderMedPos                = 1500;
    int sliderHighPos               = 2000;
    int sliderMaxPos                = 3000;
}
