package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TestMotor", group = "TeleOp")
public class MotorTest extends LinearOpMode {
    DcMotorEx testMotor;
    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        //Put name of motor you want to test in deviceName
        testMotor = hardwareMap.get(DcMotorEx.class,"FrontLeft");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setPower(0);

        telemetry.addData("Ready to start.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()){
            double motorControl = -gamepad1.left_stick_y;

            testMotor.setPower(motorControl * 0.1);

            telemetry.addData("Test Motor Tick", testMotor.getCurrentPosition());
            telemetry.addData("Time", runtime.time());
            telemetry.update();
        }
    }
}
