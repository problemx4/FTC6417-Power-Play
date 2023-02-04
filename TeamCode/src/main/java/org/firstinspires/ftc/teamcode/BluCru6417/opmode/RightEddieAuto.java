package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.Trajectories6417;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Right Side Auto Eddie")
public class RightEddieAuto extends LinearOpMode{
    int position;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize autonomous and build trajectories
        Hardware6417 robot = new Hardware6417(hardwareMap);
        robot.initCamera(hardwareMap, telemetry);

        //start robot
        robot.closeGrabber();
        robot.dropOdo();
        sleep(500);

        //build trajectory sequences
        Trajectories6417 traj = new Trajectories6417(true);

        robot.setPoseEstimate(traj.positions[0]);

        TrajectorySequence startTrajectory = traj.eddieStart(robot);
        TrajectorySequence firstParkPosition = traj.firstParkPositionBuilder(robot);
        TrajectorySequence secondParkPosition = traj.secondParkPositionBuilder(robot);
        TrajectorySequence thirdParkPosition = traj.thirdParkPositionBuilder(robot);
        TrajectorySequence parkTrajectory = secondParkPosition;

        //telemetry communication
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.isIMUCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();

        while (!isStopRequested() && opModeInInit()){
            position = robot.pipeline.position;
            telemetry.addData("Position", position);
            telemetry.addData("Red total: ", robot.pipeline.redTotal);
            telemetry.addData("Blue total: ", robot.pipeline.blueTotal);
            telemetry.addData("Green total: ", robot.pipeline.greenTotal);
            telemetry.update();
        }

        //set parking path depending on detected marker position
        switch(position){
            case 0:
                parkTrajectory = firstParkPosition;
                break;
            case 1:
                parkTrajectory = secondParkPosition;
                break;
            case 2:
                parkTrajectory = thirdParkPosition;
                break;
            default:
                break;
        }

        waitForStart(); //redundant

        //no more need for camera beyond this point
        robot.webcam.stopStreaming();

        //start off robot path
        robot.followTrajectorySequence(startTrajectory);

        while(robot.isBusy()){
            //telemetry
            telemetry.addData("Position: ", position);
            telemetry.addData("busy: ", robot.isBusy());
            telemetry.update();
        }
    }
}
