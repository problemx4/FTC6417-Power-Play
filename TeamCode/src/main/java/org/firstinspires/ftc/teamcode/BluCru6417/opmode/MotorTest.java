package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp (name = "TestMotor", group = "TeleOp")
public class MotorTest extends LinearOpMode {
    private List<DcMotorEx> testMotors = new ArrayList<>();
    ElapsedTime runtime;

    boolean LDU,LDD,LA,LB,LX,LY;



    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        DcMotorEx Slider = hardwareMap.get(DcMotorEx.class,"Slider");
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx AuxSlider = hardwareMap.get(DcMotorEx.class,"AuxSlider");
        AuxSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx RL = hardwareMap.get(DcMotorEx.class,"RearLeft");
        DcMotorEx RR = hardwareMap.get(DcMotorEx.class,"RearRight");
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx FL = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        DcMotorEx FR = hardwareMap.get(DcMotorEx.class,"FrontRight");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        //Put name of motor you want to test in deviceName

        for(DcMotorEx motor : testMotors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Ready to start.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()){
            //adding motors
            if(gamepad1.left_bumper){
                testMotors.clear();
            }

            if(gamepad1.dpad_up && !LDU){
                testMotors.add(Slider);
            }
            LDU = gamepad1.dpad_up;

            if(gamepad1.dpad_down && !LDD){
                testMotors.add(AuxSlider);
            }
            LDD = gamepad1.dpad_down;

            if(gamepad1.a && !LA){
                testMotors.add(RR);
            }
            LA = gamepad1.a;

            if(gamepad1.b && !LB){
                testMotors.add(FR);
            }
            LB = gamepad1.b;

            if(gamepad1.x && !LX){
                testMotors.add(RL);
            }
            LX = gamepad1.x;

            if(gamepad1.y && !LY){
                testMotors.add(FL);
            }
            LY = gamepad1.y;

            double motorControl = -gamepad1.left_stick_y;

            if(gamepad1.right_bumper){
                motorControl = motorControl * 0.4;
            }

            for(DcMotorEx motor : testMotors){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            for(DcMotorEx motor : testMotors){
                motor.setPower(motorControl);
            }

            for(DcMotorEx motor : testMotors){
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }

            telemetry.addData("Power", motorControl);
            telemetry.addData("Time", runtime.time());
            telemetry.update();
        }
    }
}
