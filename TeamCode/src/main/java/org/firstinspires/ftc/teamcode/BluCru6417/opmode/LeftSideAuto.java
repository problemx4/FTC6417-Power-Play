package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.Trajectories6417;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Left Side Auto")
public class LeftSideAuto extends LinearOpMode{
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
    RightSideAuto.STATE currentState;
    RightSideAuto.AUTOPATH autoPath = RightSideAuto.AUTOPATH.PARKANDCYCLE;

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
        Trajectories6417 traj = new Trajectories6417(false);

        robot.setPoseEstimate(traj.positions[0]);

        TrajectorySequence startTrajectory = traj.startAuto(robot);
        TrajectorySequence dropTrajectory = traj.dropCone(robot);
        TrajectorySequence cycleTrajectory = traj.cycle(robot);
        TrajectorySequence finalCycleTrajectory = traj.finalCycle(robot);
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

        while (!isStopRequested() && opModeInInit()){
            position = robot.pipeline.position;
            telemetry.addData("Position", position);
            telemetry.addData("Red total: ", robot.pipeline.redTotal);
            telemetry.addData("Blue total: ", robot.pipeline.blueTotal);
            telemetry.addData("Green total: ", robot.pipeline.greenTotal);
            telemetry.addData("Path?", autoPath);
            telemetry.update();

            if(gamepad1.a){
                autoPath = RightSideAuto.AUTOPATH.PARKANDCYCLE;
            }
            if(gamepad1.b){
                autoPath = RightSideAuto.AUTOPATH.PARKANDDROP;
            }
            if(gamepad1.x){
                autoPath = RightSideAuto.AUTOPATH.PARK;
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
        currentState = RightSideAuto.STATE.START;
        robot.followTrajectorySequenceAsync(startTrajectory);

        while(!isStopRequested() && opModeIsActive()){
            switch(currentState){
                case START:
                    //check if robot is done with START
                    if(!robot.isBusy()){
                        if(autoPath == RightSideAuto.AUTOPATH.PARKANDCYCLE || autoPath == RightSideAuto.AUTOPATH.PARKANDDROP){
                            currentState = RightSideAuto.STATE.DROPPING;
                            robot.followTrajectorySequenceAsync(dropTrajectory);
                        }
                        else{
                            currentState = RightSideAuto.STATE.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(ControlConstants.turretForwardPos);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                    }
                    break;
                case DROPPING:
                    //check if robot is done with DROPPING
                    if(!robot.isBusy()){
                        if(autoPath == RightSideAuto.AUTOPATH.PARKANDDROP || coneStack < 0){
                            currentState = RightSideAuto.STATE.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(ControlConstants.turretForwardPos);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                        else{
                            currentState = RightSideAuto.STATE.CYCLING;
                            robot.autoSlide(ControlConstants.sliderStackedConePos * coneStack);
                            robot.followTrajectorySequenceAsync(cycleTrajectory);
                            coneStack--;
                        }
                    }
                    break;
                case CYCLING:
                    if(!robot.isBusy()){
                        if(traj.isCyclePossible(runtime.seconds())){
                            if(coneStack == 0){
                                robot.autoSlide(ControlConstants.sliderBasePos);
                                robot.followTrajectorySequenceAsync(finalCycleTrajectory);
                            }
                            else{
                                robot.autoSlide(ControlConstants.sliderStackedConePos * coneStack);
                                robot.followTrajectorySequenceAsync(cycleTrajectory);
                                coneStack--;
                            }
                        }
                        else{
                            currentState = RightSideAuto.STATE.PARKING;
                            robot.closeGrabber();
                            robot.autoSlide(ControlConstants.sliderBasePos);
                            robot.moveWrist(ControlConstants.retractWristPos);
                            robot.autoTurret(ControlConstants.turretForwardPos);
                            robot.followTrajectorySequenceAsync(parkTrajectory);
                        }
                    }
                    break;
                case PARKING:
                    //check if robot is done parking
                    if(!robot.isBusy()){
                        currentState = RightSideAuto.STATE.IDLE;
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
            telemetry.addData("conestack: ", coneStack);
            telemetry.addData("is cycle possible: ", traj.isCyclePossible(runtime.seconds()));
            telemetry.addData("busy: ", robot.isBusy());
            telemetry.update();
        }
    }
}
