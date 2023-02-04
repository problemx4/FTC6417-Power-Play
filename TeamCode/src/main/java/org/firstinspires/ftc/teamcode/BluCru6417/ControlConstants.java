package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ControlConstants {
    /* controller variables */
    double sens                     = 0.1;
    double triggerSens              = 0.5;

    double manualServoDelta         = .01;

    /* servo variables */
    double maxDrivePower            = 0.6;
    double maxSlowerDrivePower      = 0.3;

    double highSlidePower           = 1.0;
    double medSlidePower            = 0.9;
    double lowSlidePower            = 0.8;
    double baseSlidePower           = 0.6;
    double maxManualSliderPower     = 0.7;
    double clearSlidePower          = 0.8;
    double dropConePower            = 0.5;

    double grabberClosePos          = 01.0;
    double grabberOpenPos           = 0.96;

    double retractWristPos          = 0.84;
    double lowerWristPos            = 0.39;

    double turretForwardPos         = 0.4866666;
    double turretLeftPos            = 0.16;
    double turretRightPos           = 0.8322222;

    double turretMaxPos             = turretRightPos;
    double turretMinPos             = turretLeftPos;

    double odoRetractPos            = 0.0;
    double odoDropPos               = 0.3;

    /* encoder tick variables */
    int sliderBasePos               = 0;
    int sliderStackedConePos        = 75;
    int sliderLowPos                = 750;
    int sliderMedPos                = 1250;
    int sliderHighPos               = 1750;
    int sliderMaxPos                = 1900;
    int sliderMinPos                = 0;
    int coneClearDelta              = 300;
    int armClearPos                 = 400;
    int dropConeDelta               = 250;

    /* timer variables */
    double slideDownDelay           = 0.1;
}
