package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.TrajectoriesWorlds;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Right Mid worlds auto")
public class RightMIDAuto extends LinearOpMode {
    int position;
    ElapsedTime runtime = new ElapsedTime();
    double timeLimit = 25.0;

    enum PATH{
        STARTING,
        CYCLING,
        PARKING
    }
    PATH currentPath = PATH.STARTING;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize autonomous and build trajectories
        Hardware6417 robot = new Hardware6417(hardwareMap);
        robot.initCamera(hardwareMap, telemetry);

        //start robot
        robot.closeGrabber();
        robot.dropOdo();
        robot.autoTurret(ControlConstants.turretForwardPos);
        robot.moveWrist(ControlConstants.retractWristPos);
        robot.twist(ControlConstants.twisterMidPos);
        robot.setAutoSlidePower(0.8);
        sleep(500);

        //build trajectory sequences
        TrajectoriesWorlds traj = new TrajectoriesWorlds(true, false);

        robot.setPoseEstimate(traj.positions[0]);

        TrajectorySequence startTrajectory = traj.start(robot);
        TrajectorySequence cycleTrajectory = traj.cycle(robot);
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
        while (!isStopRequested() && !robot.isIMUCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();

        while (!isStopRequested() && opModeInInit()) {
            position = robot.pipeline.position;
            telemetry.addData("Position", position);
            telemetry.addData("Red total: ", robot.pipeline.redTotal);
            telemetry.addData("Blue total: ", robot.pipeline.blueTotal);
            telemetry.addData("Green total: ", robot.pipeline.greenTotal);
            telemetry.update();
        }

        //set parking path depending on detected marker position
        switch (position) {
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
        runtime.reset();
        currentPath = PATH.STARTING;
        robot.followTrajectorySequenceAsync(startTrajectory);

        while(!isStopRequested() && opModeIsActive()){
            switch(currentPath){
                case STARTING:
                    //check if robot is done with START
                    if(!robot.isBusy()){
                        currentPath = PATH.CYCLING;
                        robot.followTrajectorySequenceAsync(cycleTrajectory);
                    }
                    break;
                case CYCLING:
                    if(!robot.isBusy()){
                        if(traj.coneStack >= 0){
                                robot.followTrajectorySequenceAsync(cycleTrajectory);
                        }
                        else{
                            currentPath = PATH.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(ControlConstants.turretForwardPos);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                    }
                    break;
                case PARKING:
                    if(!robot.isBusy()){
                       //end robot auto
                    }
            }

            //asynchronous actions
            robot.update();
            robot.updateSlide();

            //telemetry
            telemetry.addData("Position: ", position);
            telemetry.addData("Current state: ", currentPath);
            telemetry.addData("Elapsed Time: ", runtime.time());
            telemetry.addData("conestack: ", traj.coneStack);
            telemetry.addData("busy: ", robot.isBusy());
            telemetry.addData("Slide Pos", robot.slider.getCurrentPosition());
            traj.telemetry(telemetry,robot);
            telemetry.update();
        }
    }
}
