package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.PIDcontroller6417;

            /*CONTROLS
            gamepad 1:
            left stick              - holonomic drive
            right stick             - rotate
            left bumper             - slower driving (hold)
            right bumper            - toggle maintain heading
            left trigger            - max speed driving (hold)
            right trigger           - reset cumulative angle
            a (x)                   - turn correction

            gamepad 2:
            left stick              - manual move turret
            right stick             - manual move slider
            left bumper             - toggle grabber
            left trigger            - turn off limiter / turn off drop assist
            right bumper            - reset slider encoder
            right trigger           - drop cone
            a (x)                   - ground slider position
            b (circle)              - low slider position
            x (square)              - medium slider position
            y (triangle)            - high slider position
            dpadDown                - turret forward
            dpadRight               - turret left
            dpadLeft                - turret right
            dpadUP                  - toggle wrist
            */

@Config
@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class HolonomicTeleOp6417 extends LinearOpMode implements ControlConstants{

    //Enums for state machine
    enum SLIDESTATE{
        targetBase,
        autoSlide,
        manualSlide,
    }

    enum TURRETSTATE{
        forward,
        left,
        right,
        manual,
    }

    enum WRISTSTATE{
        automatic,
        manual,
    }

    public static double kP = 1.2;
    public static double kI = 0.0;
    public static double kD = 0.3;

    @Override
    public void runOpMode() {
        //initialize robot
        Hardware6417 robot = new Hardware6417(hardwareMap);

        //initialize Turn PID controller with PID constants
        PIDcontroller6417 turnPID = new PIDcontroller6417(kP,kI,kD);
        turnPID.setTolerances(Math.toRadians(5), Math.toRadians(5));

        telemetry.addData("Status", "Initialized");

        // make sure the imu gyro is calibrated before continuing.
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !robot.isIMUCalibrated()) {
            sleep(50);
            idle();
        }
        if(isStopRequested()) return;

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();

        //setup robot
        //STOP MOTORS
        robot.setWeightedDrivePower(new Pose2d(0,0,0));
        //GRAB
        robot.closeGrabber();
        //SET TURRET TO FORWARD
        robot.autoTurret(turretForwardPos);
        //Retract wrist
        robot.moveWrist(retractWristPos + 0.05);
        //Twist wrist
        robot.twist(ControlConstants.twisterMidPos);
        //retract odo
        robot.retractOdo();
        //start with angle being 180 degrees
        robot.resetAngle();
        robot.globalAngle = robot.globalAngle + Math.toRadians(180);

        //wait for start and reset timer
        waitForStart();

        //SET TURRET TO FORWARD
        robot.autoTurret(turretForwardPos);
        //Retract wrist
        robot.moveWrist(retractWristPos);

        //variables for controlling
        double verticalControl, horizontalControl, rotateControl, currentDrivePower;
        double sliderControl;

        int     slidePos = 0;
        int     dropCone = 0;
        int     slideLower = 0;

        double wristPos = lowerWristPos;

        boolean maintainHeading = true;
        boolean limiter = true;
        boolean grabbing = true;
        boolean retract = true;
        boolean lastRB1 = false;
        boolean lastLB2 = false;
        boolean lastDU2 = false;
        boolean lastF1 = false;
        boolean lastRT2 = false;
        boolean lastRB2 = false;
        boolean lastSlideDelay = false;

        ElapsedTime slideTimer = new ElapsedTime();

        SLIDESTATE slideState = SLIDESTATE.manualSlide;
        SLIDESTATE lastSlideState = SLIDESTATE.manualSlide;

        TURRETSTATE turretstate = TURRETSTATE.forward;

        WRISTSTATE wriststate = WRISTSTATE.automatic;

        //Control loop
        while(opModeIsActive()){
            //safety for switching controllers
            if(gamepad2.start || gamepad1.start){
                continue;
            }

            double heading = robot.getCumulativeAngle();

            //driving
            {
                //get gamepad joystick and dpad variables for control
                verticalControl = -1 * clipJoyInput(gamepad1.left_stick_y);
                horizontalControl = clipJoyInput(gamepad1.left_stick_x);
                rotateControl = clipJoyInput(gamepad1.right_stick_x);

                if(gamepad1.left_bumper || slidePos >= (sliderHighPos - slideLowerDelta) || robot.slider.getCurrentPosition() > sliderHighPos){
                    currentDrivePower = maxSlowerDrivePower;
                }
                else if(gamepad1.left_trigger > triggerSens){
                    currentDrivePower = 1.0;
                }
                else{
                    currentDrivePower = maxDrivePower;
                }
                if(gamepad1.a){
                    if(!lastF1){
                        turnPID.resetIntegral();
                        turnPID.setTarget(Math.PI);
                    }
                    rotateControl = -Range.clip(turnPID.calculate(heading % (2 * Math.PI)),-1.5,1.5);
                }
                lastF1 = gamepad1.a;

                //move wheels
                robot.rrHolonomicDrive(currentDrivePower,horizontalControl,verticalControl,rotateControl,maintainHeading, heading);

                //toggle maintain heading
                if(gamepad1.right_bumper && !lastRB1){
                    maintainHeading = !maintainHeading;
                }
                lastRB1 = gamepad1.right_bumper;
            }

            //slider control
            {
                sliderControl = -1 * clipJoyInput(gamepad2.right_stick_y);
                limiter = !(gamepad2.left_trigger > triggerSens);

                //if the limiter is on, enable drop assist, which is twisting grabber and lowering slides a bit
                if(limiter) {
                    slideLower = slideLowerDelta;
                }
                else{
                    slideLower = 0;
                }

                if(gamepad2.a){
                    slideState = SLIDESTATE.targetBase;
                    slidePos = sliderBasePos;
                    wristPos = lowerWristPos;
                    robot.setAutoSlidePower(baseSlidePower);
                }
                if(gamepad2.b){
                    slideState = SLIDESTATE.autoSlide;
                    slidePos = sliderLowPos;
                    wristPos = raiseWristPos;
                    robot.setAutoSlidePower(lowSlidePower);
                }
                if(gamepad2.x){
                    slideState = SLIDESTATE.autoSlide;
                    slidePos = sliderMedPos  - slideLower;
                    wristPos = raiseWristPos;
                    robot.setAutoSlidePower(medSlidePower);
                }
                if(gamepad2.y){
                    slideState = SLIDESTATE.autoSlide;
                    slidePos = sliderHighPos  - slideLower;
                    wristPos = raiseWristPos;
                    robot.setAutoSlidePower(highSlidePower);
                }
                if(Math.abs(sliderControl) >= sens){
                    slideState = SLIDESTATE.manualSlide;
                    robot.setAutoSlidePower(1.0);
                }

                //when auto moving slider move turret and wrist accordingly
                if(slideState != lastSlideState){
                    if(slideState != SLIDESTATE.manualSlide && (lastSlideState != SLIDESTATE.manualSlide || lastSlideState == SLIDESTATE.targetBase)){
                        retract = true;
                    }
                    if(slideState == SLIDESTATE.targetBase){
                        retract = true;
                        turretstate = TURRETSTATE.forward;
                        slideTimer.reset();
                    }
                }
                lastSlideState = slideState;

                //drop cone a bit
                if(gamepad2.right_trigger > triggerSens){
                    if(!lastRT2){
                        slideState = SLIDESTATE.autoSlide;
                        slidePos = robot.slider.getCurrentPosition();
                    }
                    dropCone = dropConeDelta;
                    robot.setAutoSlidePower(dropConePower);
                    wristPos = lowerWristPos;
                }
                else{
                    if(lastRT2){
                        wristPos = raiseWristPos;
                    }
                    dropCone = 0;
                }
                lastRT2 = gamepad2.right_trigger > triggerSens;

                switch(slideState){
                    case targetBase:
                        if(slideTimer.seconds() > slideDownDelay && slideTimer.seconds() < slideStallDelay){
                            robot.autoSlide(sliderBasePos);
                        }
                        if(slideTimer.seconds() > slideStallDelay){
                            if(!lastSlideDelay){
                                robot.stopSlides();
                                robot.resetSliders();
                            }
                        }
                        lastSlideDelay = slideTimer.seconds() > slideStallDelay;
                        break;
                    case autoSlide:
                        robot.autoSlide(Range.clip(slidePos - dropCone, sliderMinPos, sliderMaxPos));
                        break;
                    case manualSlide:
                        if(limiter){
                            //TWIDDLE
                            robot.autoSlide((int)(slidePos + (sliderControl * 2 * dropConeDelta)));
                        }
                        else{
                            //control slider manually
                            robot.manualSlide(maxManualSliderPower * sliderControl, limiter);
                            slidePos = robot.slider.getCurrentPosition();
                        }
                        break;
                }
            }


            //turretControl
            {
                if(gamepad2.dpad_down){
                    turretstate = TURRETSTATE.forward;
                }
                if(gamepad2.dpad_right && !gamepad2.dpad_up){
                    turretstate = TURRETSTATE.left;
                }
                if(gamepad2.dpad_left && !gamepad2.dpad_up){
                    turretstate = TURRETSTATE.right;
                }
                if(Math.abs(gamepad2.left_stick_x) > sens){
                    turretstate = TURRETSTATE.manual;
                }

                switch (turretstate){
                    case left:
                        if(robot.slider.getCurrentPosition() > armClearPos){
                            robot.autoTurret(turretLeftPos);
                        }
                        else{
                            robot.autoTurret(turretForwardPos);
                        }
                        break;
                    case right:
                        if(robot.slider.getCurrentPosition() > armClearPos){
                            robot.autoTurret(turretRightPos);
                        }
                        else{
                            robot.autoTurret(turretForwardPos);
                        }
                        break;
                    case forward:
                        if(robot.slider.getCurrentPosition() > armClearPos){
                            robot.autoTurret(turretForwardPos);
                        }
                        else{
                            robot.autoTurret(turretForwardPos);
                        }
                        break;
                    case manual:
                        if(robot.slider.getCurrentPosition() > armClearPos){
                            robot.manualTurret(clipJoyInput(gamepad2.left_stick_x));
                        }
                        else{
                            robot.autoTurret(turretForwardPos);
                        }
                        break;
                }
            }


            //wrist contol
            {
                if(gamepad2.dpad_up && !lastDU2){
                    retract = !retract;
                    wriststate = WRISTSTATE.automatic;
                }
                lastDU2 = gamepad2.dpad_up;

                if(Math.abs(gamepad2.left_stick_y) > sens){
                    retract = false;
                    wriststate = WRISTSTATE.manual;
                }

                switch(wriststate) {
                    case manual:
                        robot.manualMoveWrist(clipJoyInput(gamepad2.left_stick_y));
                        break;
                    case automatic:
                        if (retract) {
                            robot.moveWrist(retractWristPos);
                        } else {
                            robot.moveWrist(wristPos);
                        }
                        break;
                }
            }


            //twister control
            {
                //if the limiter is on and the slides are not dropped
                if(limiter && (dropCone == 0) && (robot.slider.getCurrentPosition() > (armClearPos + 250)) && (retract == false)){
                    robot.autoTwist(robot.turret.getPosition());
                }
                else{
                    robot.twist(twisterMidPos);
                }
            }


            //grabber control
            {
                if(gamepad2.left_bumper && !lastLB2){
                    grabbing = !grabbing;
                }
                lastLB2 = gamepad2.left_bumper;

                if(grabbing){
                    robot.closeGrabber();
                }
                else{
                    robot.openGrabber();
                }
            }


            //reset stuff
            {
                //reset cumulative angle
                if(gamepad1.right_trigger > triggerSens){
                    robot.resetAngle();
                    robot.globalAngle = robot.globalAngle + Math.toRadians(180);
                }
                //reset slider
                if(gamepad2.right_bumper && !lastRB2){
                    robot.resetSliders();
                }
                lastRB2 = gamepad2.right_bumper;
            }


            //telemetry
            {

                telemetry.addData("maintain heading", maintainHeading);
                telemetry.addData("SliderState", slideState);
                telemetry.addData("turret state", turretstate);
                telemetry.addData("Grabbing?", grabbing);
                telemetry.addData("Retracting?", retract);
                telemetry.addData("turn reference angle", Math.toDegrees(turnPID.target));
                telemetry.addData("pid error", turnPID.error);
                telemetry.addData("rotate control", rotateControl);
                telemetry.addData("Slide Power", robot.autoSlidePower);
                telemetry.addData("Cumulative Angle", Math.toDegrees(heading));

                robot.telemetry(telemetry);
            }
        }
        //stop robot after end
        //robot.end();
    }


    //method to make clip joystick input if it is less than sensitivity constant
    public double clipJoyInput(double input){
        if(Math.abs(input) < sens){
            return 0;
        }

        return Range.clip(input, -1, 1);
    }
}
