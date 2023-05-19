package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoriesWorlds implements ControlConstants{
    //2d array of robot positions, each row structured as follows: [x pos, y pos, robot angle]
    public Pose2d[] positions;
    public double[] angles;
    public double turretPos;
    public double firstTurretPos;
    public double firstTwistPos;
    public double wristPos;
    double twistPos;
    int slidePos;
    double turnAngle;
    public int coneStack;
    double poleXPos;
    boolean isRight;
    double rightGrabCorrection = 0.45;
    double leftGrabCorrection = 2.3;
    double rightGrabForwardCorrection = 0;
    double leftGrabForwardCorrection = 0;
    double rightCenterForwardCorrection = -1.25;
    double leftCenterForwardCorrection = -1;
    double rightCenterCorrection = -2;
    double leftCenterCorrection = -1.0;
    double rightForwardCorrection = -2;
    double leftForwardCorrection = 0.0;

    double error = 0;

    TrajectoryVelocityConstraint maxVelocity = SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    TrajectoryVelocityConstraint startVelocity = normalVelocity;
    TrajectoryVelocityConstraint cycleVelocity = normalVelocity;
    TrajectoryVelocityConstraint intakeVelocity = slowVelocity;
    TrajectoryVelocityConstraint parkVelocity = maxVelocity;

    public TrajectoriesWorlds(boolean right, boolean high){
        coneStack = 4;
        isRight = right;
        poleXPos = 24;
        if(right){
            firstTurretPos = turretRightPos;
            firstTwistPos = twisterMaxPos;
            if(high){
                turretPos = turretLeftPos;
                slidePos = sliderHighPos;
                twistPos = twisterMinPos;
                wristPos = raiseWristPos;
            }
            else{
                turretPos = turretRightPos;
                slidePos = sliderMedPos;
                twistPos = twisterMaxPos;
                wristPos = lowerWristPos;
            }
            turnAngle = Math.toRadians(90);
            //RIGHT SIDE POSITIONS
            positions = new Pose2d[]{
                    new Pose2d(36.0, -65.0 - rightForwardCorrection, Math.toRadians(-90)), //starting position       0
                    new Pose2d(36.0, -11.0, Math.toRadians(-45)),  //push pos                1
                    new Pose2d(poleXPos + rightCenterCorrection, -12.0 + rightCenterForwardCorrection, Math.toRadians(0)), //central position (in front of cone)       2
                    new Pose2d(59 + rightGrabCorrection, -12.0 + rightGrabForwardCorrection, Math.toRadians(0)), //grab cone pos           4
                    new Pose2d(35, -24.0 -rightForwardCorrection, Math.toRadians(-90)) // drop preload pos
            };
            angles = new double[]{
                    Math.toRadians(0),
                    Math.toRadians(90),
                    Math.toRadians(180),
                    Math.toRadians(-90),
            };

        }
        else{
            firstTurretPos = turretLeftPos;
            firstTwistPos = twisterMinPos;
            if(high){
                turretPos = turretRightPos;
                slidePos = sliderHighPos;
                twistPos = twisterMaxPos;
                wristPos = raiseWristPos;
            }
            else{
                turretPos = turretLeftPos;
                slidePos = sliderMedPos;
                twistPos = twisterMinPos;
                wristPos = lowerWristPos;
            }
            turnAngle = Math.toRadians(-90);
            //LEFT SIDE POSITIONS
            positions = new Pose2d[]{
                    new Pose2d(-36.0, -65.0 - leftForwardCorrection, Math.toRadians(-90)), //starting position       0
                    new Pose2d(-36.0, -11.0, Math.toRadians(-135)), //push pos                1
                    new Pose2d(-poleXPos - leftCenterCorrection, -12.0 + leftCenterForwardCorrection, Math.toRadians(-180)), //central position        2
                    new Pose2d(-59.0 - leftGrabCorrection, -12.0 + leftGrabForwardCorrection, Math.toRadians(-180)), //grab cone pos           4
                    new Pose2d(-36.0, -24.0 - leftForwardCorrection, Math.toRadians(-90))
            };
            angles = new double[]{
                    Math.toRadians(180),
                    Math.toRadians(90),
                    Math.toRadians(0),
                    Math.toRadians(-90),
            };
        }
    }

    public TrajectorySequence start(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[0])
                .setVelConstraint(startVelocity)
                .setTangent(angles[1])
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //extend slide
                    robot.autoSlide(sliderMedPos - slideLowerDelta);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //move wrist
                    robot.moveWrist(lowerWristPos);
                    //move turret over cone
                    robot.autoTurret(firstTurretPos);
                    //twist
                    robot.twist(firstTwistPos);
                })
                .splineToLinearHeading(positions[4], angles[1])
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    //dunk cone
                    robot.twist(twisterMidPos);
                    robot.openGrabber();
                    robot.autoSlide(sliderMedPos - (slideLowerDelta + (dropConeDelta * 2)));
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    //clear wrist
                    robot.moveWrist(retractWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.autoTurret(turretForwardPos);
                })
                .waitSeconds(0.5)
                .splineToLinearHeading(positions[1], angles[1])
                //CYCLE
                .setVelConstraint(cycleVelocity)
                .setTangent(angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    //move turret and wrist to grab
                    robot.moveWrist(lowerWristPos);
                    robot.autoSlide(sliderStackedConePos * coneStack);
                    coneStack--;
                })
                .splineToLinearHeading(positions[3],angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //grab
                    robot.closeGrabber();
                    //correct position
                    correctYPosition(robot, 48.03, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //raise slider
                    robot.autoSlide(slidePos - slideLowerDelta);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //move turret and twist and wrist
                    robot.autoTurret(turretPos);
                    robot.twist(twistPos);
                    robot.moveWrist(wristPos);
                })
                .setTangent(angles[2])
                .splineToLinearHeading(positions[2],angles[2])
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    //dunk cone
                    robot.twist(twisterMidPos);
                    robot.openGrabber();
                    robot.autoSlide(slidePos - (slideLowerDelta + (dropConeDelta * 2)));
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    //clear wrist
                    robot.moveWrist(retractWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.autoTurret(turretForwardPos);
                })
                .build();
    }

    public TrajectorySequence cycle(Hardware6417 robot){
        return robot.trajectorySequenceBuilder(positions[2])
                .setVelConstraint(cycleVelocity)
                .setTangent(angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    //move turret and wrist to grab
                    robot.moveWrist(lowerWristPos);
                    robot.autoSlide(sliderStackedConePos * coneStack);
                    coneStack--;
                })
                .splineToLinearHeading(positions[3],angles[0])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //grab
                    robot.closeGrabber();
                    //correct using ultrasonic
                    //TESTING ONLY ADD LEFT RIGHT FUNCTIONALITY
                    correctYPosition(robot, 48.43, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    //raise slider
                    robot.autoSlide(slidePos - slideLowerDelta);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //move turret and twist and wrist
                    robot.autoTurret(turretPos);
                    robot.twist(twistPos);
                    robot.moveWrist(wristPos);
                })
                .setTangent(angles[2])
                .splineToLinearHeading(positions[2],angles[2])
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    //correct position
                    //correctYPosition(robot,4.94, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    //dunk cone
                    robot.twist(twisterMidPos);
                    robot.openGrabber();
                    robot.autoSlide(slidePos - (slideLowerDelta + (dropConeDelta * 2)));
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    //clear wrist
                    robot.moveWrist(retractWristPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.autoTurret(turretForwardPos);
                })
                .build();
    }

    public TrajectorySequence firstParkPositionBuilder (Hardware6417 robot){
        if(isRight){
            return robot.trajectorySequenceBuilder(positions[2])
                    .setVelConstraint(maxVelocity)
                    .setTangent(angles[2])
                    .splineToLinearHeading(new Pose2d(12, -14, Math.toRadians(-90)), angles[2])
                    .build();
        }
        else{
            return robot.trajectorySequenceBuilder(positions[2])
                    .setVelConstraint(maxVelocity)
                    .setTangent(angles[0])
                    .splineToLinearHeading(new Pose2d(-60, -14, Math.toRadians(-90)), angles[0])
                    .build();
        }
    }

    public TrajectorySequence secondParkPositionBuilder (Hardware6417 robot){
        if(isRight){
            return robot.trajectorySequenceBuilder(positions[2])
                    .setVelConstraint(maxVelocity)
                    .setTangent(angles[0])
                    .splineToLinearHeading(new Pose2d(36, -14, Math.toRadians(-90)), angles[0])
                    .build();
        }
        else{
            return robot.trajectorySequenceBuilder(positions[2])
                    .setVelConstraint(maxVelocity)
                    .setTangent(angles[0])
                    .splineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(-90)), angles[0])
                    .build();
        }
    }

    public TrajectorySequence thirdParkPositionBuilder (Hardware6417 robot){
        if(isRight){
            return robot.trajectorySequenceBuilder(positions[2])
                    .setVelConstraint(maxVelocity)
                    .setTangent(angles[0])
                    .splineToLinearHeading(new Pose2d(60, -14, Math.toRadians(-90)), angles[0])
                    .build();
        }
        else{
            return robot.trajectorySequenceBuilder(positions[2])
                    .setVelConstraint(maxVelocity)
                    .setTangent(angles[2])
                    .splineToLinearHeading(new Pose2d(-12, -14, Math.toRadians(-90)), angles[2])
                    .build();
        }
    }

    public void correctYPosition(Hardware6417 robot, double cen, double min){
        double center = cen;
        double minThreshold = min;
        double maxThreshold = 10;
        error = 0;
        double measuredCenter = center;
        if(isRight == false){
            center = center + leftGrabForwardCorrection;
            measuredCenter = robot.leftRangeSensor.getDistance(DistanceUnit.INCH);
        }
        else{
            center = center + rightGrabForwardCorrection;
            measuredCenter = robot.rightRangeSensor.getDistance(DistanceUnit.INCH);
        }
        error = center - measuredCenter;
        Pose2d oldPose = robot.getPoseEstimate();
        if(Math.abs(error) > minThreshold && Math.abs(error) < maxThreshold){
            robot.setPoseEstimate(new Pose2d(oldPose.getX(), positions[3].getY() - error,oldPose.getHeading()));
        }
    }

    public void telemetry(Telemetry tele, Hardware6417 robot){
        tele.addData("error", error);
        if(isRight){
            tele.addData("sensor reading", robot.rightRangeSensor.getDistance(DistanceUnit.INCH));
        }
        else{
            tele.addData("sensor reading", robot.leftRangeSensor.getDistance(DistanceUnit.INCH));
        }
    }
}
