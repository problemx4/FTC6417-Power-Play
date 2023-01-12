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
    public int strafeAmounts[];

    public Trajectories6417(boolean right){
        if(right){
            positions = new Pose2d[]{
                    new Pose2d(36, -65, Math.toRadians(-90)), //starting position       0
                    new Pose2d(36, -10, Math.toRadians(-90)),  //push pos                1
                    new Pose2d(48, -12, Math.toRadians(-90)), //central position        2
                    new Pose2d(54, -12, Math.toRadians(-90)), //ready to grab cone pos  3
                    new Pose2d(57.5, -12, Math.toRadians(-90)), //grab cone pos           4
            };
            angles = new double[]{
                    Math.toRadians(0),
                    Math.toRadians(90),
                    Math.toRadians(180),
                    Math.toRadians(-90),
                    Math.toRadians(90),
                    Math.toRadians(-45)
            };
            strafeAmounts = new int[]{
                    -36,
                    -12,
                    12,
            };
        }
        else{
            positions = new Pose2d[]{
                    new Pose2d(-36, -65, Math.toRadians(-90)), //starting position       0
                    new Pose2d(-36, -10, Math.toRadians(-90)), //push pos                1
                    new Pose2d(-58, -12, Math.toRadians(-90)), //central position        2
                    new Pose2d(-54, -12, Math.toRadians(-90)), //ready to grab cone pos  3
                    new Pose2d(-57.5, -12, Math.toRadians(-90)), //grab cone pos           4
            };
            angles = new double[]{
                    Math.toRadians(180),
                    Math.toRadians(90),
                    Math.toRadians(0),
                    Math.toRadians(-90),
                    Math.toRadians(-90),
                    Math.toRadians(225)
            };
            strafeAmounts = new int[]{
                    -36,
                    -12,
                    12,
            };
        }
    }

    TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), 12.3);
    TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(90), 12.3);

    public TrajectorySequence startAuto(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[0])
                .setVelConstraint(normalVelocity)
                .setTangent(angles[1])
                .splineToSplineHeading(positions[1], angles[1])
                .setTangent(angles[5])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.autoSlide(sliderLowPos);
                    robot.autoTurret(0);
                })
                .splineToLinearHeading(positions[2], angles[2])
                .build();
    }

    public TrajectorySequence dropCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                .setVelConstraint(slowVelocity)
                .forward(0.0001)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //lower wrist
                    robot.moveWrist(lowerWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                    //drop cone
                    robot.openGrabber();
                })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //close grabber
                    robot.closeGrabber();
                    robot.autoTurret(angles[4]);
                })
                .back(0.0001)
                .build();
    }

    public TrajectorySequence grabCone(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
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
                .waitSeconds(0.3)
                .build();
    }

    public TrajectorySequence returnDrop(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[4])
                .setVelConstraint(normalVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    //raise slider to drop cone
                    robot.autoSlide(sliderLowPos);
                    //retract wrist
                    robot.moveWrist(retractWristPos);
                })
                .waitSeconds(0.1)
                .setTangent(angles[2])
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    //move turret to drop cone
                    robot.autoTurret(0);
                })
                .splineToSplineHeading(positions[2], angles[2])
                .build();
    }

    public TrajectorySequence returnPark(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[4])
                .setVelConstraint(normalVelocity)
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> {
                    //clear cone
                    robot.autoSlide(robot.slider.getCurrentPosition() + coneClearDelta);
                    //retract wrist
                    robot.moveWrist(retractWristPos);
                })
                .waitSeconds(0.5)
                .setTangent(angles[2])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //move turret to drop cone
                    robot.autoTurret(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                    //lower slide to base position
                    robot.autoSlide(sliderBasePos);
                })
                .splineToSplineHeading(positions[2], angles[2])
                .build();
    }

    public TrajectorySequence firstParkPositionBuilder (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                .strafeLeft(strafeAmounts[0])
                .build();
    }

    public TrajectorySequence secondParkPositionBuilder (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                .strafeLeft(strafeAmounts[1])
                .build();
    }

    public TrajectorySequence thirdParkPositionBuilder (Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                .strafeLeft(strafeAmounts[2])
                .build();
    }


    public boolean isCyclePossible(double elapsedTime){
        double cycleTime = 8;
        double parkTime = 3;
        double autoTime = 30;

        //check if the amount of time left is more than the time it takes to cycle
        return (autoTime - (elapsedTime + parkTime)) > cycleTime;
    }
}
