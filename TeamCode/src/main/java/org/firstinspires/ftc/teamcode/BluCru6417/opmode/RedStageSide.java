package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.Trajectories6417;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Stage side")
public class RedStageSide extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    int position;
    enum STATE{
        START,
        DROPPING,
        GRABBING,
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

        robot.closeGrabber();
        sleep(500);
        robot.retractWrist();

        //build trajectory sequences
        robot.setPoseEstimate(Trajectories6417.positions[0]);

        TrajectorySequence startTrajectory = Trajectories6417.startAuto(robot);
        TrajectorySequence dropTrajectory = Trajectories6417.dropCone(robot);
        TrajectorySequence grabTrajectory = Trajectories6417.grabCone(robot);
        TrajectorySequence firstParkPosition = Trajectories6417.firstParkPositionBuilder(robot);
        TrajectorySequence secondParkPosition = Trajectories6417.secondParkPositionBuilder(robot);
        TrajectorySequence thirdParkPosition = Trajectories6417.thirdParkPositionBuilder(robot);
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
                        if((Trajectories6417.isCyclePossible(runtime.time()) && autoPath == AUTOPATH.PARKANDCYCLE) || autoPath == AUTOPATH.PARKANDDROP){
                            currentState = STATE.DROPPING;
                            robot.followTrajectorySequenceAsync(dropTrajectory);
                        }
                        else{
                            currentState = STATE.PARKING;
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                    }
                    break;
                case DROPPING:
                    //check if robot is done with DROPPING
                    if(!robot.isBusy()){
                        if(autoPath == AUTOPATH.PARKANDDROP){
                            currentState = STATE.PARKING;
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                        else{
                            currentState = STATE.GRABBING;
                            robot.autoSlide(ControlConstants.sliderStackedConePos * coneStack);
                            robot.autoWrist();
                            robot.followTrajectorySequenceAsync(grabTrajectory);
                            coneStack--;
                        }
                    }
                    break;
                case GRABBING:
                    //check if grabbing is done
                    if(!robot.isBusy()){
                        if(Trajectories6417.isCyclePossible(runtime.time()) && autoPath == AUTOPATH.PARKANDCYCLE && !(coneStack < 0)){
                            currentState = STATE.DROPPING;
                            robot.followTrajectorySequenceAsync(dropTrajectory);
                        }
                        else{
                            currentState = STATE.PARKING;
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                    }
                    break;
                case PARKING:
                    //check if robot is done parking
                    if(!robot.isBusy()){
                        robot.autoWrist();
                        robot.autoArm(ControlConstants.armServoDownPos);
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
