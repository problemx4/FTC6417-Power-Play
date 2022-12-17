package org.firstinspires.ftc.teamcode.BluCru6417;

public interface ControlConstants {
    /* controller variables */
    double sens                     = 0.1;
    double triggerSens              = 0.5;

    double manualServoDelta         = 0.01;

    /* servo variables */
    double maxDrivePower            = 0.6;
    double maxSlowerDrivePower      = 0.3;

    double maxSliderPower           = 0.4;
    double autoSlidePower           = 0.8;
    double clearSlidePower          = 0.9;

    double grabberClosePos          = 1.0;
    double grabberOpenPos           = 0.87;
    
    double turretMaxPos             = 1.0;
    double turretMinPos             = 0.0;

    double turretForwardPos         = 0.5;
    double turretLeftPos            = 0.1;
    double turretRightPos           = 0.9;

    /* encoder tick variables */
    int sliderBasePos               = 0;
    int sliderStackedConePos        = 110;
    int sliderLowPos                = 300;
    int sliderMedPos                = 1200;
    int sliderHighPos               = 2100;
    int sliderMaxPos                = 3000;
    int coneClearDelta              = 600;
}
