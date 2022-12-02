package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;

            /*CONTROLS
            gamepad 1:
            left stick              - holonomic drive
            right stick             - rotate
            left bumper             - slower driving (hold)
            right bumper            - toggle maintain heading
            left trigger            - max speed driving (hold)
            right trigger           - reset cumulative angle
            a (x)                   - close grabber
            b (circle)              - open grabber
            dpad down               - turn on autoWrist
            dpad left               - manual wrist forward
            dpad right              - manual wrist back

            gamepad 2:
            vertical left stick     - manual move arm
            vertical right stick    - manual move slider
            left bumper             - toggle grabber
            right bumper            - reset slider encoder
            right trigger           - turn off limiter
            a (x)                   - ground slider position
            b (circle)              - low slider position
            x (square)              - medium slider position
            y (triangle)            - high slider position
            dpad down               - manual wrist forward
            dpad right              - retract wrist
            dpad left               - full backward arm position
            dpad up                 - manual wrist back
            */


@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class HolonomicTeleOp6417 extends LinearOpMode implements ControlConstants{

    ElapsedTime armTimer = new ElapsedTime();
    ElapsedTime slideTimer = new ElapsedTime();

    //Enums for state machine
    enum WRISTSTATE{
        autoWrist,
        manualWrist,
        retractWrist
    }
    enum SLIDESTATE{
        targetBase,
        targetLow,
        targetMed,
        targetHigh,
        manualSlide,
    }

    enum ARMSTATE{
        manualArm,
        targetDown,
        targetBack,
    }


    @Override
    public void runOpMode() {
        //initialize robot
        Hardware6417 robot = new Hardware6417(hardwareMap);
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
        robot.start();

        //wait for start and reset timer
        waitForStart();
        armTimer.reset();
        slideTimer.reset();

        //variables for controlling
        double verticalControl, horizontalControl, rotateControl, currentDrivePower;
        double sliderControl;
        double armControl;

        boolean maintainHeading = true;
        boolean lastRB1 = false;
        boolean lastDR2 = false;
        boolean lastLB2 = false;
        boolean limiter = true;
        boolean grabbing = true;
        boolean lastSlideBusy = false;

        SLIDESTATE slideState = SLIDESTATE.manualSlide;
        SLIDESTATE lastSlideState = SLIDESTATE.manualSlide;
        WRISTSTATE wristState = WRISTSTATE.autoWrist;
        ARMSTATE armState = ARMSTATE.targetDown;
        ARMSTATE lastArmState = ARMSTATE.targetDown;


        //Control loop
        while(opModeIsActive()){
            //safety for switching controllers
            if(gamepad2.start || gamepad1.start){
                continue;
            }

            //driving
            {
                //get gamepad joystick and dpad variables for control
                verticalControl = -1 * clipJoyInput(gamepad1.left_stick_y);
                horizontalControl = clipJoyInput(gamepad1.left_stick_x);
                rotateControl = clipJoyInput(gamepad1.right_stick_x);

                if(gamepad1.left_bumper || slideState == SLIDESTATE.targetHigh || robot.leftSlider.getCurrentPosition() > sliderHighPos || robot.rightSlider.getCurrentPosition() > sliderHighPos){
                    currentDrivePower = maxSlowerDrivePower;
                }
                else if(gamepad1.left_trigger > triggerSens){
                    currentDrivePower = 1.0;
                }
                else{
                    currentDrivePower = maxDrivePower;
                }

                //move wheels
                robot.rrHolonomicDrive(currentDrivePower,horizontalControl,verticalControl,rotateControl,maintainHeading);

                //toggle maintain heading
                if(gamepad1.right_bumper && !lastRB1){
                    maintainHeading = !maintainHeading;
                }
                lastRB1 = gamepad1.right_bumper;
            }

            if(grabbing){
                //slider control
                {
                    sliderControl = -1 * clipJoyInput(gamepad2.right_stick_y);
                    limiter = !(gamepad2.right_trigger > triggerSens);

                    if(gamepad2.a){
                        slideState = SLIDESTATE.targetBase;
                    }
                    if(gamepad2.b){
                        slideState = SLIDESTATE.targetLow;
                    }
                    if(gamepad2.x){
                        slideState = SLIDESTATE.targetMed;
                    }
                    if(gamepad2.y){
                        slideState = SLIDESTATE.targetHigh;
                    }
                    if(Math.abs(sliderControl) >= sens){
                        slideState = SLIDESTATE.manualSlide;
                    }

                    if(slideState == SLIDESTATE.targetBase && lastSlideState != SLIDESTATE.targetBase){
                        armState = ARMSTATE.targetDown;
                        lastArmState = ARMSTATE.manualArm;
                        robot.clearSliders(armClearDelta);
                        slideTimer.reset();
                    }
                    lastSlideState = slideState;

                    if(slideTimer.seconds() > sliderDelay){
                        switch(slideState){
                            case targetBase:
                                robot.autoSlide(sliderBasePos);
                                break;
                            case targetLow:
                                robot.autoSlide(sliderLowPos);
                                break;
                            case targetMed:
                                robot.autoSlide(sliderMedPos);
                                break;
                            case targetHigh:
                                robot.autoSlide(sliderHighPos);
                                break;
                            case manualSlide:
                                //control slider manually
                                robot.manualSlide(maxSliderPower * sliderControl, limiter);
                                break;
                        }

                        if((!robot.leftSlider.isBusy() || !robot.rightSlider.isBusy()) && lastSlideBusy && (wristState == WRISTSTATE.retractWrist) && slideTimer.seconds() > sliderRetractDelay){
                            wristState = WRISTSTATE.autoWrist;
                        }
                        lastSlideBusy = robot.leftSlider.isBusy() || robot.rightSlider.isBusy();
                    }
                }


                //arm control
                {
                    armControl = -1 * clipJoyInput(gamepad2.left_stick_y);

                    //check each button and set target position
                    //servo arm code
                    if(gamepad2.dpad_left){
                        armState = ARMSTATE.targetBack;
                    }
                    if(Math.abs(armControl) > sens){
                        armState = ARMSTATE.manualArm;
                        lastArmState = ARMSTATE.manualArm;
                    }

                    if(armState != lastArmState){
                        wristState = WRISTSTATE.retractWrist;
                        armTimer.reset();
                    }
                    lastArmState = armState;

                    if(armTimer.seconds() > armDelay){
                        switch(armState){
                            case targetDown:
                                robot.autoArm(ControlConstants.armServoDownPos);
                                break;
                            case targetBack:
                                robot.autoArm(ControlConstants.armServoBackwardsPos);
                                break;
                            case manualArm:
                                robot.manualArm(manualServoDelta * armControl);
                                break;
                        }
                    }
                }


                //wrist control
                {
                    switch(wristState){
                        case autoWrist:
                            robot.autoWrist();
                            //case to switch state
                            if(gamepad2.dpad_right){
                                wristState = WRISTSTATE.retractWrist;
                            }
                            else if(gamepad1.dpad_right || gamepad1.dpad_left || gamepad2.dpad_down || gamepad2.dpad_up){
                                wristState = WRISTSTATE.manualWrist;
                            }
                            break;
                        case manualWrist:
                            //move forward
                            if(gamepad1.dpad_right || gamepad2.dpad_down){
                                robot.manualWrist(-manualServoDelta);
                            }
                            //move back
                            else if(gamepad1.dpad_left || gamepad2.dpad_up){
                                robot.manualWrist(manualServoDelta);
                            }
                            //cases to swtich state
                            if(gamepad2.dpad_right){
                                wristState = WRISTSTATE.retractWrist;
                            }
                            if(gamepad1.dpad_down){
                                wristState = WRISTSTATE.autoWrist;
                            }
                            break;
                        case retractWrist:
                            //retract wrist (clutch)
                            robot.retractWrist();
                            //case to switch state
                            if((!gamepad2.dpad_right && lastDR2) || gamepad1.dpad_down){
                                wristState = WRISTSTATE.autoWrist;
                            }
                            lastDR2 = gamepad2.dpad_right;
                            break;
                    }
                }
            }

            //grabber control
            {
                if(armTimer.seconds() > armFinishDelay && slideTimer.seconds() > sliderFinishDelay && ((!robot.leftSlider.isBusy() || !robot.rightSlider.isBusy()) || slideState == SLIDESTATE.manualSlide)){
                    if(gamepad1.a){
                        grabbing = true;
                    }
                    if(gamepad1.b){
                        grabbing = false;
                    }
                    if(gamepad2.left_bumper && !lastLB2){
                        grabbing = !grabbing;
                    }
                    lastLB2 = gamepad2.left_bumper;

                    if(grabbing){
                        robot.closeGrabber();
                    }
                    else{
                        robot.openGrabber();
                        if(slideState == SLIDESTATE.manualSlide){
                            robot.manualSlide(0,true);
                        }
                    }
                }
            }


            //reset stuff
            {
                //reset cumulative angle
                if(gamepad1.right_trigger > triggerSens){
                    robot.resetAngle();
                }
                //reset slider
                if(gamepad2.right_bumper){
                    robot.resetSliders();
                }
            }


            //telemetry
            {
                telemetry.addData("maintain heading", maintainHeading);
                telemetry.addData("ArmState", armState);
                telemetry.addData("WristState", wristState);
                telemetry.addData("SliderState", slideState);
                telemetry.addData("Grabbing?", grabbing);

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
