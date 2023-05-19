package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;
import org.firstinspires.ftc.teamcode.BluCru6417.Hardware6417;
import org.firstinspires.ftc.teamcode.BluCru6417.PIDcontroller6417;
import org.firstinspires.ftc.teamcode.BluCru6417.PIDslider6417;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp(name = "Odo Tester", group = "TeleOp")
public class OdoTester extends LinearOpMode {

    SampleMecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize motors to be tuned and loop to control them
        robot = new SampleMecanumDrive(hardwareMap);

        DcMotorEx leftEncoder = hardwareMap.get(DcMotorEx.class, "AuxSlider");
        DcMotorEx rightEncoder = hardwareMap.get(DcMotorEx.class, "RightEncoder");



        waitForStart();

        while(opModeIsActive()){
            robot.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_y));

            telemetry.addData("Left current position", leftEncoder.getCurrentPosition());
            telemetry.addData("Right current position", rightEncoder.getCurrentPosition());
            telemetry.update();

            if(gamepad1.a){
                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}