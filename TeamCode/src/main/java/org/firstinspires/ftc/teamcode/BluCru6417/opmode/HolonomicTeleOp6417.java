package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
            left trigger            - toggle turret field centric movement
            right bumper            - reset slider encoder
            right trigger           - turn off limiter
            a (x)                   - ground slider position
            b (circle)              - low slider position
            x (square)              - medium slider position
            y (triangle)            - high slider position
            dpadDown                - turret forward
            dpadRight               - turret left
            dpadLeft                - turret right
            dpadUP                  - toggle wrist
            */


@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class HolonomicTeleOp6417 extends LinearOpMode implements ControlConstants{

    //Enums for state machine
    enum SLIDESTATE{
        targetBase,
        targetLow,
        targetMed,
        targetHigh,
        manualSlide,
    }

    enum TURRETSTATE{
        forward,
        left,
        right,
        manual,
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
        robot.setWeightedDrivePower(new Pose2d(0,0,0));
        //GRAB
        robot.closeGrabber();
        //SET TURRET TO FORWARD
        robot.autoTurret(0);
        //Retract wrist
        robot.moveWrist(retractWristPos);

        //wait for start and reset timer
        waitForStart();

        //variables for controlling
        double verticalControl, horizontalControl, rotateControl, currentDrivePower;
        double sliderControl;
        double turretAngleOffset;
        double turretReferenceAngle = 0;

        boolean maintainHeading = true;
        boolean offsetTurret = false;
        boolean lastOffsetTurret = false;
        boolean limiter = true;
        boolean grabbing = true;
        boolean retract = true;
        boolean lastRB1 = false;
        boolean lastLB2 = false;
        boolean lastLT2 = false;
        boolean lastDU2 = false;

        ElapsedTime slideTimer = new ElapsedTime();

        SLIDESTATE slideState = SLIDESTATE.manualSlide;
        SLIDESTATE lastSlideState = SLIDESTATE.manualSlide;

        TURRETSTATE turretstate = TURRETSTATE.forward;

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

                if(gamepad1.left_bumper || slideState == SLIDESTATE.targetHigh || robot.slider.getCurrentPosition() > sliderHighPos){
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
                    robot.slider.setTargetPosition(robot.slider.getTargetPosition() + 1);
                }

                if(slideState != lastSlideState){
                    retract = true;
                    if(slideState == SLIDESTATE.targetBase){
                        turretstate = TURRETSTATE.forward;
                        slideTimer.reset();
                    }
                }
                lastSlideState = slideState;

                switch(slideState){
                    case targetBase:
                        if(slideTimer.seconds() > slideDownDelay){
                            robot.autoSlide(sliderBasePos);
                        }
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
                if(offsetTurret){
                    turretAngleOffset = robot.getCumulativeAngle() - turretReferenceAngle;
                }
                else{
                    turretAngleOffset = 0;
                }

                if(offsetTurret && !lastOffsetTurret){
                    turretReferenceAngle = robot.getCumulativeAngle();
                }
                lastOffsetTurret = offsetTurret;

                if((gamepad2.left_trigger > triggerSens) && !lastLT2){
                    offsetTurret = !offsetTurret;
                }
                lastLT2 = gamepad2.left_trigger > triggerSens;

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
                        robot.autoTurret((Math.PI / 2.0) - turretAngleOffset);
                        break;
                    case right:
                        robot.autoTurret((-Math.PI / 2.0) - turretAngleOffset);
                        break;
                    case forward:
                        robot.autoTurret((0.0) - turretAngleOffset);
                        break;
                    case manual:
                        robot.manualTurret(clipJoyInput(gamepad2.left_stick_x));
                        break;
                }
            }


            //wrist contol
            {
                if(gamepad2.dpad_up && !lastDU2){
                    retract = !retract;
                }
                lastDU2 = gamepad2.dpad_up;

                if(retract){
                    robot.moveWrist(retractWristPos);
                }
                else{
                    robot.moveWrist(lowerWristPos);
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
                telemetry.addData("turret state", turretstate);
                telemetry.addData("TurretOffset?", offsetTurret);
                telemetry.addData("Grabbing?", grabbing);
                telemetry.addData("Retracting?", retract);
                telemetry.addData("offsetAngle", turretAngleOffset);
                telemetry.addData("reference angle", turretReferenceAngle);

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
