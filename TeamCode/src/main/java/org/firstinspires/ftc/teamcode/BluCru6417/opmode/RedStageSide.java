package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.Trajectories6417;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Stage side")
public class RedStageSide extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    int position;
    enum STATE{
        START,
        CYCLING,
        PARKING,
        IDLE
    }
    STATE currentState;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize autonomous and build trajectories
        Hardware6417 robot = new Hardware6417(hardwareMap);
        robot.initCamera(hardwareMap, telemetry);

        robot.start();

        //build trajectory sequences
        robot.setPoseEstimate(new Pose2d(36, -65, Math.toRadians(90)));

        TrajectorySequence redStageTrajectory = Trajectories6417.redStageSideBuilder(robot);
        TrajectorySequence cycleTrajectory = Trajectories6417.redStageCycleBuilder(robot);
        Trajectory firstParkPosition = Trajectories6417.firstParkPositionBuilder(robot);
        Trajectory secondParkPosition = Trajectories6417.secondParkPositionBuilder(robot);
        Trajectory thirdParkPosition = Trajectories6417.thirdParkPositionBuilder(robot);
        Trajectory parkTrajectory = secondParkPosition;

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

        while (opModeInInit()){
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
        runtime.reset();

        //start off robot path
        currentState = STATE.CYCLING;
        robot.followTrajectorySequenceAsync(redStageTrajectory);

        while(!isStopRequested() && opModeIsActive()){
            switch(currentState){
                case CYCLING:
                    //check if robot is done with CYClING
                    if(!robot.isBusy()){
                        if(Trajectories6417.isCyclePossible(runtime.time())){
                            currentState = STATE.CYCLING;
                            robot.followTrajectorySequenceAsync(cycleTrajectory);
                        }
                        else{
                            currentState = STATE.PARKING;
                            robot.followTrajectoryAsync(parkTrajectory);
                        }
                    }
                    break;
                case PARKING:
                    //check if robot is done parking
                    if(!robot.isBusy()){
                        currentState = STATE.IDLE;
                    }
                case IDLE:
                    break;
            }

            //asynchronous actions
            robot.update();
            //slider.update();?

            //telemetry
            telemetry.addData("Position: ", position);
            telemetry.addData("Current state: ", currentState);
            telemetry.addData("Elapsed Time: ", runtime.time());
            telemetry.update();
        }
    }
}
