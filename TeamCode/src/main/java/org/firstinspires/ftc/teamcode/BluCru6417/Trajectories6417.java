package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories6417 implements ControlConstants{
    //2d array of robot positions, each row structured as follows: [x pos, y pos, robot angle]
    public Pose2d[] positions;
    public double[] angles;
    boolean isRight;
    double rightGrabCorrection = 0;
    double leftGrabCorrection = 0;
    double rightCenterCorrection = 0;
    double leftCenterCorrection = 0;
    double rightForwardCorrection = 0;
    double leftForwardCorrection = 0;

    public Trajectories6417(boolean right){
        isRight = right;
        if(right){
            positions = new Pose2d[]{
                    new Pose2d(36, -63.5 - rightForwardCorrection, Math.toRadians(-90)), //starting position       0
                    new Pose2d(36, -12, Math.toRadians(-90)),  //push pos                1
                    new Pose2d(48 + rightCenterCorrection, -12, Math.toRadians(-90)), //central position        2
                    new Pose2d(54, -12, Math.toRadians(-90)), //ready to grab cone pos  3
                    new Pose2d(58 + rightGrabCorrection, -12, Math.toRadians(-90)), //grab cone pos           4
            };
            angles = new double[]{
                    Math.toRadians(0),
                    Math.toRadians(90),
                    Math.toRadians(180),
                    Math.toRadians(-90),
                    turretLeftPos,
                    Math.toRadians(0)
            };
        }
        else{
            positions = new Pose2d[]{
                    new Pose2d(-36, -63.5 - leftForwardCorrection, Math.toRadians(-90)), //starting position       0
                    new Pose2d(-36, -12, Math.toRadians(-90)), //push pos                1
                    new Pose2d(-48 - leftCenterCorrection, -12, Math.toRadians(-90)), //central position        2
                    new Pose2d(-54, -12, Math.toRadians(-90)), //ready to grab cone pos  3
                    new Pose2d(-58 - leftGrabCorrection, -12, Math.toRadians(-90)), //grab cone pos           4
            };
            angles = new double[]{
                    Math.toRadians(180),
                    Math.toRadians(90),
                    Math.toRadians(0),
                    Math.toRadians(-90),
                    turretRightPos,
                    Math.toRadians(180)
            };
        }
    }

    TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 12.3);
    TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(12, Math.toRadians(90), 12.3);

    public TrajectorySequence startAuto(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[0])
                .setVelConstraint(normalVelocity)
                .setTangent(angles[1])
                .splineToSplineHeading(positions[1], angles[1])
                .setTangent(angles[5])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.autoSlide(sliderLowPos);
                    robot.autoTurret(turretForwardPos);
                })
                .setVelConstraint(slowVelocity)
                .splineToLinearHeading(positions[2], angles[2])
                .build();
    }

    public TrajectorySequence dropCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                .setVelConstraint(slowVelocity)
                .waitSeconds(0.001)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //lower wrist
                    robot.moveWrist(lowerWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    //drop cone
                    robot.openGrabber();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //close grabber
                    robot.closeGrabber();
                    robot.autoTurret(angles[4]);
                })
                .waitSeconds(0.001)
                .build();
    }

    public TrajectorySequence cycle(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                //GRABBING CONE
                .setVelConstraint(slowVelocity)
                .setTangent(angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //lower slider (maybe not necessary and to be used outside in auto class)
                    //move turret to grab
                    robot.autoTurret(angles[4]);
                    robot.openGrabber();
                })
                .splineToSplineHeading(positions[4], angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.closeGrabber();
                })
                .waitSeconds(0.2)
                //RETURNING TO DROP
                .setVelConstraint(normalVelocity)
                .setTangent(angles[2])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider to drop cone
                    robot.autoSlide(sliderLowPos);
                    //retract wrist
                    //robot.moveWrist(retractWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    //move turret to drop cone
                    robot.autoTurret(turretForwardPos);
                })
                .waitSeconds(0.15)
                .splineToSplineHeading(positions[2], angles[2])
                //DROPPING CONE
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //lower wrist
                    robot.moveWrist(lowerWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    //drop cone
                    robot.openGrabber();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //close grabber
                    robot.closeGrabber();
                    robot.autoTurret(angles[4]);
                })
                .waitSeconds(0.2)
                .build();
    }

    public TrajectorySequence finalCycle(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                //GRABBING CONE
                .setVelConstraint(slowVelocity)
                .setTangent(angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //lower slider (maybe not necessary and to be used outside in auto class)
                    //move turret to grab
                    robot.autoTurret(angles[4]);
                    robot.moveWrist(ControlConstants.lowerWristPos - 0.01);
                    robot.openGrabber();
                })
                .splineToSplineHeading(positions[4], angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //grab cone
                    robot.closeGrabber();
                })
                .waitSeconds(0.2)
                //RETURNING TO DROP
                .setVelConstraint(normalVelocity)
                .setTangent(angles[2])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //raise slider to drop cone
                    robot.autoSlide(sliderLowPos + 200);
                    //retract wrist
                    //robot.moveWrist(retractWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    //move turret to drop cone
                    robot.autoTurret(turretForwardPos);
                })
                .waitSeconds(0.3)
                .splineToSplineHeading(positions[2], angles[2])
                //DROPPING CONE
                .setVelConstraint(slowVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //lower wrist
                    robot.moveWrist(lowerWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    //drop cone
                    robot.openGrabber();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //close grabber
                    robot.closeGrabber();
                    robot.autoTurret(angles[4]);
                })
                .waitSeconds(0.2)
                .build();
    }

    public TrajectorySequence firstParkPositionBuilder (Hardware6417 robot){
        if(isRight){
            return robot.trajectorySequenceBuilder(positions[2])
                    .strafeRight(36)
                    .build();
        }
        else{
            return robot.trajectorySequenceBuilder(positions[2])
                    .strafeRight(12)
                    .build();
        }
    }

    public TrajectorySequence secondParkPositionBuilder (Hardware6417 robot){
        if(isRight){
            return robot.trajectorySequenceBuilder(positions[2])
                    .strafeRight(12)
                    .build();
        }
        else{
            return robot.trajectorySequenceBuilder(positions[2])
                    .strafeLeft(12)
                    .build();
        }
    }

    public TrajectorySequence thirdParkPositionBuilder (Hardware6417 robot){
        if(isRight){
            return robot.trajectorySequenceBuilder(positions[2])
                    .strafeLeft(12)
                    .build();
        }
        else{
            return robot.trajectorySequenceBuilder(positions[2])
                    .strafeLeft(36)
                    .build();
        }
    }

    public TrajectorySequence eddieStart (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(new Pose2d(36, -63.5, Math.toRadians(-90)))
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(60, -63.5, Math.toRadians(-90)), 0)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(60, -18, Math.toRadians(-90)), Math.toRadians(90))
                .turn(Math.toRadians(26))
                .build();
    }


    public boolean isCyclePossible(double elapsedTime){
        double cycleTime = 4;
        double parkTime = 1;
        double autoTime = 30;

        //check if the amount of time left is more than the time it takes to cycle
        return (autoTime - (elapsedTime + parkTime)) > cycleTime;
    }
}
