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

            gamepad 2:
            left stick              - manual move turret
            right stick             - manual move slider
            left bumper             - toggle grabber
            right bumper            - reset slider encoder
            right trigger           - turn off limiter
            a (x)                   - ground slider position
            b (circle)              - low slider position
            x (square)              - medium slider position
            y (triangle)            - high slider position
            dpadUP                  - turret forward
            dpadLeft                - turret left
            dpadRight               - turret right
            */


@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class HolonomicTeleOp6417 extends LinearOpMode implements ControlConstants{

    ElapsedTime slideTimer = new ElapsedTime();

    //Enums for state machine
    enum SLIDESTATE{
        targetBase,
        targetLow,
        targetMed,
        targetHigh,
        manualSlide,
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
        //STOP MOTORS
        //GRAB
        //SET TURRET TO FORWARD

        //wait for start and reset timer
        waitForStart();
        slideTimer.reset();

        //variables for controlling
        double verticalControl, horizontalControl, rotateControl, currentDrivePower;
        double sliderControl;

        boolean maintainHeading = true;
        boolean lastRB1 = false;
        boolean lastLB2 = false;
        boolean limiter = true;
        boolean grabbing = true;

        SLIDESTATE slideState = SLIDESTATE.manualSlide;
        SLIDESTATE lastSlideState = SLIDESTATE.manualSlide;

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
                    robot.autoTurret(turretForwardPos);
                    slideTimer.reset();
                }
                lastSlideState = slideState;

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
            }


            //turretControl
            {
                if(gamepad2.dpad_up){
                    robot.autoTurret(turretForwardPos);
                }
                if(gamepad2.dpad_left){
                    robot.autoTurret(turretLeftPos);
                }
                if(gamepad2.dpad_right){
                    robot.autoTurret(turretRightPos);
                }

                if(gamepad2.left_stick_y > sens){
                    robot.manualTurret(gamepad2.left_stick_y);
                }
            }


            //grabber control
            {
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
