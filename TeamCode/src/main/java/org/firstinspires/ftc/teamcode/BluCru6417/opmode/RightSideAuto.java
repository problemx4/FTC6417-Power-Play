package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.Trajectories6417;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Right Side Auto")
public class RightSideAuto extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    boolean finished = false;
    int position;
    enum STATE{
        START,
        DROPPING,
        CYCLING,
        PARKING,
        IDLE
    }
    enum AUTOPATH{
        PARK,
        PARKANDDROP,
        PARKANDCYCLE,
    }
    STATE currentState;
    AUTOPATH autoPath = AUTOPATH.PARKANDCYCLE;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize autonomous and build trajectories
        Hardware6417 robot = new Hardware6417(hardwareMap);
        robot.initCamera(hardwareMap, telemetry);

        //start robot
        robot.closeGrabber();
        robot.autoTurret(0);
        sleep(500);

        //build trajectory sequences
        Trajectories6417 traj = new Trajectories6417(true);

        robot.setPoseEstimate(traj.positions[0]);

        TrajectorySequence startTrajectory = traj.startAuto(robot);
        TrajectorySequence dropTrajectory = traj.dropCone(robot);
        TrajectorySequence cycleTrajectory = traj.cycle(robot);
        TrajectorySequence firstParkPosition = traj.firstParkPositionBuilder(robot);
        TrajectorySequence secondParkPosition = traj.secondParkPositionBuilder(robot);
        TrajectorySequence thirdParkPosition = traj.thirdParkPositionBuilder(robot);
        TrajectorySequence parkTrajectory = secondParkPosition;

        int coneStack = 4;

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
            telemetry.addData("Path?", autoPath);
            telemetry.update();

            if(gamepad1.a){
                autoPath = AUTOPATH.PARKANDCYCLE;
            }
            if(gamepad1.b){
                autoPath = AUTOPATH.PARKANDDROP;
            }
            if(gamepad1.x){
                autoPath = AUTOPATH.PARK;
            }
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
        runtime.reset();
        currentState = STATE.START;
        robot.followTrajectorySequenceAsync(startTrajectory);

        while(!isStopRequested() && opModeIsActive()){
            switch(currentState){
                case START:
                    //check if robot is done with START
                    if(!robot.isBusy()){
                        if(autoPath == AUTOPATH.PARKANDCYCLE || autoPath == AUTOPATH.PARKANDDROP){
                            currentState = STATE.DROPPING;
                            robot.followTrajectorySequenceAsync(dropTrajectory);
                        }
                        else{
                            currentState = STATE.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(0);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                    }
                    break;
                case DROPPING:
                    //check if robot is done with DROPPING
                    if(!robot.isBusy()){
                        if(autoPath == AUTOPATH.PARKANDDROP || coneStack < 0){
                            currentState = STATE.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(0);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                        else{
                            currentState = STATE.CYCLING;
                            robot.autoSlide(ControlConstants.sliderStackedConePos * coneStack);
                            robot.followTrajectorySequenceAsync(cycleTrajectory);
                            coneStack--;
                        }
                    }
                    break;
                case CYCLING:
                    if(!robot.isBusy()){
                        if(traj.isCyclePossible(runtime.seconds())){
                            robot.autoSlide(ControlConstants.sliderStackedConePos * coneStack);
                            robot.followTrajectorySequenceAsync(cycleTrajectory);
                            coneStack--;
                        }
                        else{
                            currentState = STATE.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(0);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
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
            telemetry.addData("Path:", autoPath);
            telemetry.addData("Elapsed Time: ", runtime.time());
            telemetry.update();
        }
    }
}
