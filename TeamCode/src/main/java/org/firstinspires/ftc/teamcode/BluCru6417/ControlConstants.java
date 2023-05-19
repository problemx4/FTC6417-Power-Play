package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ControlConstants {
    /* controller variables */
    double sens                     = 0.1;
    double triggerSens              = 0.5;

    double manualServoDelta         = .01;

    /* servo variables */
    double maxDrivePower            = 0.8;
    double maxSlowerDrivePower      = 0.15;

    double highSlidePower           = 1.0;
    double medSlidePower            = 1.0;
    double lowSlidePower            = 1.0;
    double baseSlidePower           = 0.5;
    double maxManualSliderPower     = 0.7;
    double clearSlidePower          = 0.8;
    double dropConePower            = 0.8;

    double grabberClosePos          = 1.0;
    double grabberOpenPos           = 0.8;

    double retractWristPos          = 0.07;
    double dropWristPos             = 1.0;
    double lowerWristPos            = 0.78;
    double raiseWristPos            = 0.7;

    double twisterMinPos            = 0.485;
    double twisterMidPos            = 0.511666;
    double twisterMaxPos            = 0.542777;

    double autoTwistSens            = 0.2;

    double turretForwardPos         = 0.5;
    double turretLeftPos            = 0.96;
    double turretRightPos           = 0.04;

    double turretMaxPos             = turretLeftPos;
    double turretMinPos             = turretRightPos;

    double leftOdoRetractPos            = 0.0;
    double leftOdoDropPos               = 0.3;
    double rightOdoRetractPos            = 0.0;
    double rightOdoDropPos               = 0.3;

    /* encoder tick variables */
    int sliderBasePos               = 0;
    int sliderStackedConePos        = 40;
    int sliderLowPos                = 425;
    int sliderMedPos                = 815;
    int sliderHighPos               = 1115;
    int sliderMaxPos                = 1100;
    int armClearPos                 = 300;
    int sliderMinPos                = 0;
    int coneClearDelta              = 180;
    int dropConeDelta               = 75;
    int slideLowerDelta             = 65;

    /* timer variables */
    double slideDownDelay           = 0.15;
    double slideStallDelay          = 1.5;
}
