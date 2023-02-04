package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
        MeepMeep meepMeep = new MeepMeep(400);

        TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 12.3);
        TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(90), 12.3);

        RoadRunnerBotEntity startAuto = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(36, 30, Math.toRadians(120), Math.toRadians(180), 12.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -63.5, Math.toRadians(-90)))
                                .setTangent(0)
                                .splineToSplineHeading(new Pose2d(60, -63.5, Math.toRadians(-90)), 0)
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(60, -18, Math.toRadians(-64)), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity dropCone = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(36, 30, Math.toRadians(120), Math.toRadians(180), 12.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24, -12, Math.toRadians(-90)))
                                .setVelConstraint(slowVelocity)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //lower wrist
                                })
                                .forward(5)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //drop cone
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //retract wrist
                                    //close grabber
                                })
                                .back(5)
                                .build()
                );

        RoadRunnerBotEntity grabCone = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(36, 30, Math.toRadians(120), Math.toRadians(180), 12.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24, -12, Math.toRadians(-90)))
                                .setVelConstraint(normalVelocity)
                                .setTangent(0)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //lower slider (maybe not necessary and to be used outside in auto class)
                                    //move turret to grab
                                })
                                .splineTo(new Vector2d(54, -12), 0)
                                .setVelConstraint(slowVelocity)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //open grabber
                                    //lower wrist
                                })
                                .splineTo(new Vector2d(60, -12), 0)
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //grab cone
                                })
                                .build()
                );

        RoadRunnerBotEntity returnToDropCone = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(36, 30, Math.toRadians(120), Math.toRadians(180), 12.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, -12, Math.toRadians(-90)))
                                .setVelConstraint(normalVelocity)
                                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                                    //raise slider to drop cone
                                    //retract wrist
                                })
                                .waitSeconds(0.3)
                                .setTangent(Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //move turret to drop cone
                                })
                                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                                .build()
                );

        RoadRunnerBotEntity returnToPark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(36, 30, Math.toRadians(120), Math.toRadians(180), 12.5)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, -12, Math.toRadians(-90)))
                                .setVelConstraint(normalVelocity)
                                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                                    //clear cone
                                    //retract wrist
                                })
                                .waitSeconds(0.3)
                                .setTangent(Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                                    //move turret to drop cone
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                                    //lower slide to base position
                                })
                                .splineTo(new Vector2d(36, -12), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(startAuto)
                .addEntity(dropCone)
                .addEntity(grabCone)
                .addEntity(returnToDropCone)
                .addEntity(returnToPark)
                .start();
    }
}