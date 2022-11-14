package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
            left trigger            - even slower driving (hold)
            right trigger           - reset cumulative angle
            a (x)                   - close grabber
            b (circle)              - open grabber
            dpad down               - turn on autoWrist
            dpad left               - manual wrist back
            dpad                    - manual wrist forward

            gamepad 2:
            vertical left stick     - manual move arm
            vertical right stick    - manual move slider
            left bumper             - reset arm encoder
            right bumper            - reset slider encoder
            a (x)                   - ground slider position
            b (circle)              - low slider position
            x (square)              - medium slider position
            y (triangle)            - high slider position
            dpad down               - down arm position
            dpad right              - full forward arm position
            dpad left               - full backward arm position
            dpad up                 - useful grabbing arm position
            */



@TeleOp(name = "Holonomic TeleOp", group = "TeleOp")
public class HolonomicTeleOp6417 extends LinearOpMode implements ControlConstants{

    ElapsedTime runtime = new ElapsedTime();

    //Enums for state machine
    enum DRIVESTATE{
        joyDrive,
        dpadDrive
    }
    enum ARMSTATE{
        autoArm,
        manualArm
    }
    enum WRISTSTATE{
        autoWrist,
        manualWrist
    }
    enum SLIDESTATE{
        autoSlide,
        manualSlide
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
        runtime.reset();

        //variables for controlling
        double verticalControl, horizontalControl, rotateControl, currentDrivePower;

        double sliderControl;

        double armControl;
        boolean joyArmActive;

        boolean maintainHeading = true;

        boolean lastRB1 = false;


        DRIVESTATE driveState = DRIVESTATE.joyDrive;
        SLIDESTATE slideState = SLIDESTATE.autoSlide;
        ARMSTATE armState = ARMSTATE.autoArm;
        WRISTSTATE wristState = WRISTSTATE.autoWrist;



        //Control loop
        while(opModeIsActive()){
            //driving
            {
                //get gamepad joystick and dpad variables for control
                verticalControl = -1 * clipJoyInput(gamepad1.left_stick_y);
                horizontalControl = clipJoyInput(gamepad1.left_stick_x);
                rotateControl = clipJoyInput(gamepad1.left_stick_x);

                if(gamepad1.left_bumper){
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



            //slider control
            {
                sliderControl = -1 * clipJoyInput(gamepad2.right_stick_y);

                switch(slideState){
                    case autoSlide:
                        //check each button and set target position
                        if(gamepad2.a){
                            robot.autoSlide(sliderBasePos);
                        }
                        if(gamepad2.b){
                            robot.autoSlide(sliderLowPos);
                        }
                        if(gamepad2.x){
                            robot.autoSlide(sliderMedPos);
                        }
                        if(gamepad2.y){
                            robot.autoSlide(sliderHighPos);
                        }
                        //case to change state
                        if(Math.abs(sliderControl) >= sens){
                            slideState = SLIDESTATE.manualSlide;
                        }
                        break;
                    case manualSlide:
                        //control slider manually
                        robot.manualSlide(maxSliderPower * sliderControl);

                        //case to change state (any button are pressed)
                        if(gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y){
                            slideState = SLIDESTATE.autoSlide;
                        }
                        break;
                }
            }



            //arm control
            {
                armControl = -1 * clipJoyInput(gamepad2.left_stick_y);

                //necessary because of maintaining arm
                joyArmActive = Math.abs(armControl) >= sens;

                switch (armState){
                    case autoArm:
                        //check each button and set target position
                        //servo arm code
                        if(gamepad2.dpad_down){
                            robot.autoArm(armServoDownPos);
                        }
                        if(gamepad2.dpad_right){
                            robot.autoArm(armServoForwardPos);
                        }
                        if(gamepad2.dpad_left){
                            robot.autoArm(armServoBackwardsPos);
                        }
                        if(gamepad2.dpad_up){
                            robot.autoArm(armServoUsefulPos);
                        }

                        //case to change state
                        if(joyArmActive /* || !robot.arm.isBusy()  // only necessary for maintain arm*/){
                            armState = ARMSTATE.manualArm;
                        }
                        break;
                    case manualArm:
                        //servo arm code
                        if(joyArmActive){
                            robot.manualArm(manualWristDelta * armControl);
                        }
                        else{
                            //robot.maintainArm(); necessary?
                        }

                        //case to change state
                        if(gamepad2.dpad_down || gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_up){
                            armState = ARMSTATE.autoArm;
                        }
                }
            }



            //wrist control
            {
                switch(wristState){
                    case autoWrist:
                        robot.autoWrist();
                        //case to switch state
                        if(gamepad1.dpad_right || gamepad1.dpad_left){
                            wristState = WRISTSTATE.manualWrist;
                        }
                        break;
                    case manualWrist:
                        //move forward
                        if(gamepad1.dpad_right){
                            robot.manualWrist(-manualWristDelta);
                        }
                        //move back
                        if(gamepad1.dpad_left){
                            robot.manualWrist(manualWristDelta);
                        }
                        //case to swtich state
                        if(gamepad1.dpad_down){
                            wristState = WRISTSTATE.autoWrist;
                        }
                        break;
                }
            }



            //grabber control
            {
                if(gamepad1.a){
                    robot.closeGrabber();
                }
                if(gamepad1.b){
                    robot.openGrabber();
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

                robot.telemetry(telemetry);
            }
        }
        //stop robot after end
        robot.end();
    }

    //method to make clip joystick input if it is less than sensitivity constant
    public double clipJoyInput(double input){
        if(Math.abs(input) < sens){
            return 0;
        }

        return Range.clip(input, -1, 1);
    }
}
