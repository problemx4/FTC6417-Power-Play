package org.firstinspires.ftc.teamcode.BluCru6417;

public interface ControlConstants {
    //angle variables
    double armServoVerticalPosition = 0.86;
    double armServoHorizontalPosition = 0.5;

    double wristVerticalPosition    = .5;
    double wristHorizontalPosition  = 0.43555;


    /* control variables */
    //power values
    double maxDrivePower            = 0.6;
    double maxSlowerDrivePower      = 0.3;

    double maxSliderPower           = 0.4;
    double autoSlidePower           = 0.8;
    double clearSlidePower          = 0.9;

    //sensitivity values
    double sens                     = 0.1;
    double triggerSens              = 0.5;

    //servo values
    double manualServoDelta         = 0.01;

    double grabberClosePos          = 1.0;
    double grabberOpenPos           = 0.87;

    double armServoDownPos          = 0.92;
    double armServoForwardPos       = 1.0;
    double armServoBackwardsPos     = armServoHorizontalPosition;
    double armServoUsefulPos        = armServoDownPos;
    double armStartPos              = 0.9;

    //encoder tick variables
    int sliderBasePos               = 0;
    int sliderStackedConePos        = 100;
    int sliderLowPos                = 300;
    int sliderMedPos                = 1200;
    int sliderHighPos               = 2100;
    int sliderMaxPos                = 3000;
    int armClearDelta               = 700;
    int coneClearDelta              = 600;

    //timing variables
    double armDelay                 = 0.2;
    double armFinishDelay           = armDelay + 0.3;
    double sliderDelay              = armFinishDelay;
    double sliderRetractDelay       = sliderDelay + 0.1;
    double sliderFinishDelay        = sliderDelay + 2.0;
}
