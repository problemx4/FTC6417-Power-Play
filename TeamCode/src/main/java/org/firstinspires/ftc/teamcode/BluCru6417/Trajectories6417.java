package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories6417 implements ControlConstants{
    static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 12.3);
    static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(90), 12.3);

    static double droppingCorrection = 0; // positive for further
    static double startCorrection = 1.0; // positive for closer
    static double grabbingCorrection = -0.5; // positive for further into wall

    //2d array of robot positions, each row structured as follows: [x pos, y pos, robot angle]
    public static Pose2d[] rightPositions = {
            new Pose2d(36, -65 + startCorrection, Math.toRadians(90)), //starting position
            new Pose2d(36, -12, Math.toRadians(45)), //central position
            new Pose2d(54, -12, Math.toRadians(0)),  //ready to grab cone position
            new Pose2d(36, -2.5, Math.toRadians(90)),  //push cone beyond
            new Pose2d(36, -12, Math.toRadians(0)), //after dropping cone
            new Pose2d(36, -7, Math.toRadians(90)), //push cone beyond if only parking
    };

    public static Pose2d[] leftPositions = {
            new Pose2d(-36, -65 + startCorrection, Math.toRadians(90)), //starting position
            new Pose2d(-36, -12, Math.toRadians(135)), //central position
            new Pose2d(-54, -12, Math.toRadians(180)),  //ready to grab cone position
            new Pose2d(-36, -2.5, Math.toRadians(90)),  //push cone beyond
            new Pose2d(-36, -12, Math.toRadians(180)), //after dropping cone
            new Pose2d(-36, -7, Math.toRadians(90)), //push cone beyond if only parking
    };

    public static TrajectorySequence rightStartAuto(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(rightPositions[0])
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(rightPositions[3], Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(rightPositions[1], Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .build();
    }

    public static TrajectorySequence leftStartAuto(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(leftPositions[0])
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(leftPositions[3], Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(leftPositions[1], Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .build();
    }

    public static TrajectorySequence rightDropCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(rightPositions[1])
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //raise slider and prepare it to place
                    robot.autoSlide(sliderMedPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //move arm
                })
                .waitSeconds(1)
                .back(11 + droppingCorrection)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                    robot.openGrabber();
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    //drop cone
                    robot.closeGrabber();
                })
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    //drop slider
                })
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //drop slider
                    robot.autoSlide(sliderBasePos);
                })
                .splineToSplineHeading(rightPositions[4], Math.toRadians(45))
                .build();
    }

    public static TrajectorySequence leftDropCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(leftPositions[1])
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //raise slider and prepare it to place
                    robot.autoSlide(sliderMedPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //move arm
                })
                .waitSeconds(1)
                .back(11 + droppingCorrection)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //drop cone
                    robot.openGrabber();
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    //drop cone
                    robot.closeGrabber();
                })
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    //drop slider
                })
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //drop slider
                    robot.autoSlide(sliderBasePos);
                })
                .splineToSplineHeading(leftPositions[4], Math.toRadians(135))
                .build();
    }

    public static TrajectorySequence rightGrabCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(rightPositions[4])
                //grabbing cone segment, raise slider to grab cone in autonomous
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(rightPositions[2],Math.toRadians(0))
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.openGrabber();
                })
                .forward(8 + grabbingCorrection)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.closeGrabber();
                })
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    // raise slider off stack
                    robot.clearSliders(coneClearDelta);
                })
                .waitSeconds(0.3)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    robot.autoSlide(sliderBasePos);
                })
                .splineToSplineHeading(rightPositions[1], Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence leftGrabCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(leftPositions[4])
                //grabbing cone segment, raise slider to grab cone in autonomous
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(leftPositions[2],Math.toRadians(180))
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.openGrabber();
                })
                .forward(8 + grabbingCorrection)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.closeGrabber();
                })
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    // raise slider off stack
                    robot.clearSliders(coneClearDelta);
                })
                .waitSeconds(0.3)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    robot.autoSlide(sliderBasePos);
                })
                .splineToSplineHeading(leftPositions[1], Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence firstParkPositionBuilder (Hardware6417 robot, boolean right){
        int multiplier = -1;
        Pose2d[] poses = leftPositions;
        if(right){
            multiplier = 1;
            poses = rightPositions;
        }

        return robot.trajectorySequenceBuilder(poses[1])
                .turn(Math.toRadians(45 * multiplier))
                .strafeLeft(24)
                .build();
    }

    public static TrajectorySequence secondParkPositionBuilder (Hardware6417 robot, boolean right){
        int multiplier = -1;
        Pose2d[] poses = leftPositions;
        if(right){
            multiplier = 1;
            poses = rightPositions;
        }

        return robot.trajectorySequenceBuilder(poses[1])
                .turn(Math.toRadians(45 * multiplier))
                .build();
    }

    public static TrajectorySequence thirdParkPositionBuilder (Hardware6417 robot, boolean right){
        int multiplier = -1;
        Pose2d[] poses = leftPositions;
        if(right){
            multiplier = 1;
            poses = rightPositions;
        }

        return robot.trajectorySequenceBuilder(poses[1])
                .turn(Math.toRadians(45 * multiplier))
                .strafeRight(24)
                .build();
    }


    public static boolean isCyclePossible(double elapsedTime){
        double cycleTime = 8;
        double parkTime = 3;
        double autoTime = 30;

        //check if the amount of time left is more than the time it takes to cycle
        return (autoTime - (elapsedTime + parkTime)) > cycleTime;
    }
}
