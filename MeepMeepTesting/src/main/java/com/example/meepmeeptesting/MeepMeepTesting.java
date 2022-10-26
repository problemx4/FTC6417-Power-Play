package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity redStageSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(36, 30, Math.toRadians(120), Math.toRadians(180), 12.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -65, Math.toRadians(90)))
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(36, -48), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d( 36, -12,Math.toRadians(225)), Math.toRadians(90))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(120), 10))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //raise slider and prepare it to place
                                })
                                .forward(6)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //drop cone
                                })
                                .resetVelConstraint()
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)), Math.toRadians(45))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //drop slider
                                })
                                //CYCLE SECTION: 8 sec
                                .setTangent(Math.toRadians(0))
                                .splineTo(new Vector2d(56, -12), Math.toRadians(0))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(120), 10))
                                .forward(7)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //grab cone

                                })
                                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                                    // raise slider off stack
                                })
                                .waitSeconds(.5)
                                .resetVelConstraint()
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(36,-12,Math.toRadians(225)), Math.toRadians(180))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(120), 10))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //raise slider and prepare it to place
                                })
                                .forward(6)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //drop cone
                                })
                                .resetVelConstraint()
                                .setTangent(Math.toRadians(45))
                                .splineToSplineHeading(new Pose2d(36, -12, Math.toRadians(0)), Math.toRadians(45))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //drop slider
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redStageSide)
                .start();
    }
}