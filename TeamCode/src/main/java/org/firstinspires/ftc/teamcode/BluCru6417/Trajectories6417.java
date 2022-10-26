package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories6417 {
    public static TrajectorySequence redStageSideBuilder (SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(new Pose2d(36, -65, Math.toRadians(90)))
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
                .build();
    }

    public static TrajectorySequence redStageCycleBuilder (SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(new Pose2d(36,-12, Math.toRadians(0)))//CYCLE SECTION: 8 sec
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
                .waitSeconds(1)
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
                .build();
    }
    public static TrajectorySequence redAudienceSideBuilder (SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-12, -53), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12, -36,Math.toRadians(135)), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(180)), Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop slider
                })
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-24,-36), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12.5))
                .forward(6)
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //grab cone after running into it for .5 sec
                })
                .splineToSplineHeading(new Pose2d(-36,-36, Math.toRadians(45)), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12.5))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-36,-36, Math.toRadians(90)), Math.toRadians(-135))
                .build();
    }

    public static TrajectorySequence blueStageSideBuilder (SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(new Pose2d(36, 65, Math.toRadians(-90)))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12, 53), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(12, 36,Math.toRadians(-45)), Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(-225))
                .splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(0)), Math.toRadians(-225))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop slider
                })
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24,36), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12.5))
                .forward(6)
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //grab cone after running into it for .5 sec
                })
                .splineToSplineHeading(new Pose2d(36,36, Math.toRadians(-135)), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12.5))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(36,36, Math.toRadians(-90)), Math.toRadians(45))
                .build();
    }

    public static TrajectorySequence blueAudienceSideBuilder (SampleMecanumDrive drive){
        return drive.trajectorySequenceBuilder(new Pose2d(-36, 65, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-12, 53), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-12, 36,Math.toRadians(-135)), Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(-180)), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop slider
                })
                .setTangent(Math.toRadians(-180))
                .splineTo(new Vector2d(-24,36), Math.toRadians(-180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12.5))
                .forward(6)
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //grab cone after running into it for .5 sec
                })
                .splineToSplineHeading(new Pose2d(-36,36, Math.toRadians(-45)), Math.toRadians(-180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 12.5))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider and prepare it to place
                })
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //drop cone
                })
                .resetVelConstraint()
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(-36,36, Math.toRadians(-90)), Math.toRadians(135))
                .build();
    }

    public static Trajectory firstParkPositionBuilder (SampleMecanumDrive drive){
        return drive.trajectoryBuilder(new Pose2d(36,-12, Math.toRadians(0)), Math.toRadians(0))
                .back(24)
                .build();
    }

    public static Trajectory secondParkPositionBuilder (SampleMecanumDrive drive){
        return drive.trajectoryBuilder(new Pose2d(36,-12, Math.toRadians(0)), Math.toRadians(0))
                .forward(1)
                .build();
    }

    public static Trajectory thirdParkPositionBuilder (SampleMecanumDrive drive){
        return drive.trajectoryBuilder(new Pose2d(36,-12, Math.toRadians(0)), Math.toRadians(0))
                .forward(24)
                .build();
    }

    public static boolean isCyclePossible(double elapsedTime){
        double cycleTime = 8;
        double parkTime = 1;
        double autoTime = 30;

        //check if the amount of time left is more than the time it takes to cycle
        return (autoTime - (elapsedTime + parkTime)) > cycleTime;
    }
}
