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
            left bumper             - open grabber
            right bumper            - close grabber
            left trigger            - manual move wrist back
            right trigger           - manual move wrist forward
            a (x)                   - toggle maintain heading
            b (circle)              - reset cumulative angle
            x (square)              - turn on autoWrist
            dpad                    - slower driving

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

        //lolol

        //variables for controlling
        double verticalControl, horizontalControl, rotateControl, verticalSlowControl, horizontalSlowControl;

        double sliderControl;

        double armControl;
        boolean joyArmActive;

        boolean maintainHeading = true;

        boolean lastA1 = false;


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

                verticalSlowControl = getGamepad1DpadY();
                horizontalSlowControl = getGamepad1DpadX();

                switch(driveState){
                    case joyDrive:
                        //move wheels
                        robot.rrHolonomicDrive(maxDrivePower,horizontalControl,verticalControl,rotateControl,maintainHeading);

                        //condition to change state
                        if((getGamepad1DpadX() != 0)|| (getGamepad1DpadY() != 0)){
                            driveState = DRIVESTATE.dpadDrive;
                        }
                        break;
                    case dpadDrive:
                        //move wheels slowly
                        robot.rrHolonomicDrive(maxSlowerDrivePower,horizontalSlowControl,verticalSlowControl,rotateControl,maintainHeading);

                        //condition to change state
                        if((Math.abs(verticalControl) >= sens) || (Math.abs(horizontalControl) >= sens)){
                            driveState = DRIVESTATE.joyDrive;
                        }
                        break;
                }

                //toggle maintain heading
                if(gamepad1.a && !lastA1){
                    maintainHeading = !maintainHeading;
                }
                lastA1 = gamepad1.a;
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
                        if(gamepad2.dpad_down){
                            robot.autoArm(armDownPos);
                        }
                        if(gamepad2.dpad_right){
                            robot.autoArm(armForwardPos);
                        }
                        if(gamepad2.dpad_left){
                            robot.autoArm(armBackwardsPos);
                        }
                        if(gamepad2.dpad_up){
                            robot.autoArm(armUsefulPos);
                        }
                        //case to change state
                        if(joyArmActive /* || !robot.arm.isBusy()  // only necessary for maintain arm*/){
                            armState = ARMSTATE.manualArm;
                        }
                        break;
                    case manualArm:
                        if(joyArmActive){
                            robot.manualArm(maxArmPower * armControl);
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
                        if(gamepad1.right_trigger > triggerSens || gamepad1.left_trigger > triggerSens){
                            wristState = WRISTSTATE.manualWrist;
                        }
                        break;
                    case manualWrist:
                        if(gamepad1.right_trigger > triggerSens){
                            robot.manualWrist(-manualWristDelta);
                        }
                        if(gamepad1.left_trigger > triggerSens){
                            robot.manualWrist(manualWristDelta);
                        }
                        //case to swtich state
                        if(gamepad1.x){
                            wristState = WRISTSTATE.autoWrist;
                        }
                        break;
                }
            }



            //grabber control
            {
                if(gamepad1.right_bumper){
                    robot.closeGrabber();
                }
                if(gamepad1.left_bumper){
                    robot.openGrabber();
                }
            }



            //reset stuff
            {
                //reset cumulative angle
                if(gamepad1.b){
                    robot.resetAngle();
                }
                //reset slider
                if(gamepad2.right_bumper){
                    robot.resetSliders();
                }
                //reset arm
                if(gamepad2.left_bumper){
                    robot.resetArm();
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

    //methods to convert dpad from individual buttons into something similar to joystick inputs
    public double getGamepad1DpadX(){
        if(gamepad1.dpad_right){
            return 1.0;
        }
        if(gamepad1.dpad_left){
            return -1.0;
        }

        return 0.0;
    }

    public double getGamepad1DpadY(){
        if(gamepad1.dpad_up){
            return 1.0;
        }
        if(gamepad1.dpad_down){
            return -1.0;
        }

        return 0.0;
    }
}
