package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TestMotor", group = "TeleOp")
public class MotorTest extends LinearOpMode {
    DcMotorEx testMotor1, testMotor2, testMotor3, testMotor4;
    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        //Put name of motor you want to test in deviceName
        testMotor1 = hardwareMap.get(DcMotorEx.class,"One");
        testMotor2 = hardwareMap.get(DcMotorEx.class,"Two");
        testMotor3 = hardwareMap.get(DcMotorEx.class,"Three");
        testMotor4 = hardwareMap.get(DcMotorEx.class,"Four");


        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor1.setPower(0);

        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setPower(0);

        testMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor3.setPower(0);

        testMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor4.setPower(0);

        telemetry.addData("Ready to start.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()){
            double motorControl = -gamepad1.left_stick_y;

            testMotor1.setPower(motorControl);
            testMotor2.setPower(motorControl);
            testMotor3.setPower(motorControl);
            testMotor4.setPower(motorControl);

            //telemetry.addData("Test Motor Tick", testMotor1.getCurrentPosition());
            //telemetry.addData("Test Motor Tick", testMotor2.getCurrentPosition());
            //telemetry.addData("Test Motor Tick", testMotor3.getCurrentPosition());
            //telemetry.addData("Test Motor Tick", testMotor4.getCurrentPosition());

            telemetry.addData("Time", runtime.time());
            telemetry.update();
        }
    }
}
