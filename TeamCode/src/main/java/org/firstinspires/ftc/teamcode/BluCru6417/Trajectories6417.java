package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories6417 implements ControlConstants{
    static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 12.3);
    static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(90), 12.3);

    //2d array of robot positions, each row structured as follows: [x pos, y pos, robot angle]
    public static Pose2d[] positions = {
            new Pose2d(36, -64, Math.toRadians(90)), //starting position
            new Pose2d(36, -12, Math.toRadians(45)), //central position
            new Pose2d(54, -12, Math.toRadians(0)),  //ready to grab cone position
            new Pose2d(36, 0, Math.toRadians(90)),  //push cone beyond
            new Pose2d(36, -12, Math.toRadians(0)), //after dropping cone
    };

    public static TrajectorySequence startAuto(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[0])
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(positions[3], Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(positions[1], Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.retractWrist();
                })
                .build();
    }

    public static TrajectorySequence dropCone (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[1])
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //raise slider and prepare it to place
                    robot.autoSlide(sliderMedPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //move arm
                    robot.autoArm(armServoBackwardsPos);
                    robot.autoWrist();
                })
                .waitSeconds(1)
                .back(10)
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
                    robot.autoArm(armServoDownPos);
                    robot.clearSliders(armClearDelta);
                })
                .UNSTABLE_addTemporalMarkerOffset(.5,() -> {
                    //drop slider
                    robot.autoSlide(sliderBasePos);
                    robot.retractWrist();
                })
                .splineToSplineHeading(positions[4], Math.toRadians(45))
                .build();
    }

    public static TrajectorySequence grabCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[4])
                //grabbing cone segment, raise slider to grab cone in autonomous
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(positions[2],Math.toRadians(0))
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.openGrabber();
                })
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.closeGrabber();
                })
                .UNSTABLE_addTemporalMarkerOffset(.2,() -> {
                    // raise slider off stack
                    robot.clearSliders(coneClearDelta);
                })
                .waitSeconds(0.2)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    robot.autoSlide(sliderBasePos);
                    robot.retractWrist();
                })
                .splineToSplineHeading(positions[1], Math.toRadians(180))
                .build();
    }


    public static TrajectorySequence firstParkPositionBuilder (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[1])
                .turn(Math.toRadians(45))
                .strafeLeft(24)
                .build();
    }

    public static TrajectorySequence secondParkPositionBuilder (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[1])
                .turn(Math.toRadians(45))
                .build();
    }

    public static TrajectorySequence thirdParkPositionBuilder (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[1])
                .turn(Math.toRadians(45))
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
