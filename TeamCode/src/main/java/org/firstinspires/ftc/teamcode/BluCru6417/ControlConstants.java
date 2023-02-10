package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ControlConstants {
    /* controller variables */
    double sens                     = 0.1;
    double triggerSens              = 0.5;

    double manualServoDelta         = .01;

    /* servo variables */
    double maxDrivePower            = 0.75;
    double maxSlowerDrivePower      = 0.3;

    double highSlidePower           = 0.8;
    double medSlidePower            = 0.8;
    double lowSlidePower            = 0.8;
    double baseSlidePower           = 0.7;
    double maxManualSliderPower     = 0.7;
    double clearSlidePower          = 0.8;
    double dropConePower            = 0.5;

    double grabberClosePos          = 1.0;
    double grabberOpenPos           = 0.96;

    double retractWristPos          = 0.95;
    double lowerWristPos            = 0.5;

    double turretForwardPos         = 0.4866666;
    double turretLeftPos            = 0.16;
    double turretRightPos           = 0.8322222;

    double turretMaxPos             = turretRightPos;
    double turretMinPos             = turretLeftPos;

    double odoRetractPos            = 0.0;
    double odoDropPos               = 0.3;

    /* encoder tick variables */
    int sliderBasePos               = 0;
    int sliderStackedConePos        = 45;
    int sliderLowPos                = 500;
    int sliderMedPos                = 800;
    int sliderHighPos               = 1150;
    int sliderMaxPos                = 1350;
    int sliderMinPos                = 0;
    int coneClearDelta              = 180;
    int armClearPos                 = 120;
    int dropConeDelta               = 145;

    /* timer variables */
    double slideDownDelay           = 0.15;
    double slideStallDelay          = 1.5;
}
