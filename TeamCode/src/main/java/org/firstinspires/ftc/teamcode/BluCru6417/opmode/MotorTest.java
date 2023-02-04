package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp (name = "TestMotor", group = "TeleOp")
public class MotorTest extends LinearOpMode {
    private List<DcMotorEx> testMotors;
    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        //Put name of motor you want to test in deviceName
        testMotors.add(hardwareMap.get(DcMotorEx.class,"One"));
        testMotors.add(hardwareMap.get(DcMotorEx.class,"Two"));
        testMotors.add(hardwareMap.get(DcMotorEx.class,"Three"));
        testMotors.add(hardwareMap.get(DcMotorEx.class,"Four"));

        for(DcMotorEx motor : testMotors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }

        telemetry.addData("Ready to start.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()){
            double motorControl = -gamepad1.left_stick_y;

            for(DcMotorEx motor : testMotors){
                motor.setPower(motorControl);
            }

            for(DcMotorEx motor : testMotors){
                telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            }

            telemetry.addData("Time", runtime.time());
            telemetry.update();
        }
    }
}
