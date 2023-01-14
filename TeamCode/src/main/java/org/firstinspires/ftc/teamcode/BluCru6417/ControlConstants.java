package org.firstinspires.ftc.teamcode.BluCru6417;

public interface ControlConstants {
    /* controller variables */
    double sens                     = 0.1;
    double triggerSens              = 0.5;

    double manualServoDelta         = .01;

    /* servo variables */
    double maxDrivePower            = 0.6;
    double maxSlowerDrivePower      = 0.3;

    double maxSliderPower           = 0.4;
    double autoSlidePower           = 0.9;
    double clearSlidePower          = 0.9;

    double grabberClosePos          = 0.995;
    double grabberOpenPos           = 0.955;
    
    double turretMaxPos             = 1.0;
    double turretMinPos             = 0.0;

    double retractWristPos          = 0.45;
    double lowerWristPos            = 0.06;

    double turretForwardPos         = 0.464444;
    double turretLeftPos            = 0.81;
    double turretRightPos           = 0.13;

    /* encoder tick variables */
    int sliderBasePos               = 0;
    int sliderStackedConePos        = 110;
    int sliderLowPos                = 1300;
    int sliderMedPos                = 2150;
    int sliderHighPos               = 3050;
    int sliderMaxPos                = 4000;
    int sliderMinPos                = 0;
    int coneClearDelta              = 600;

    /* timer variables */
    double slideDownDelay           = 0.1;
}
