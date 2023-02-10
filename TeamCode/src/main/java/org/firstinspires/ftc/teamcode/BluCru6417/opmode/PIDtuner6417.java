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
import org.firstinspires.ftc.teamcode.BluCru6417.PIDcontroller6417;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "PIDTuner", group = "TeleOp")
public class PIDtuner6417 extends LinearOpMode {
    public enum pathState{
        Increasing,
        Decreasing,
        Base,
        Peak,
    }

    public static double kP = 1.61;
    public static double kI = 0;
    public static double kD = 0.01;
    public static double tunerA = 0.999;

    public static double distance = Math.PI / 10.0;

    private ElapsedTime timer = new ElapsedTime();
    public static double delay = 4.0;

    PIDcontroller6417 controller = new PIDcontroller6417(kP,kI,kD);

    SampleMecanumDrive drive;

    public pathState path = pathState.Base;

    public void init(HardwareMap hwMap){
        //initialize whatever is to be tuned
        drive = new SampleMecanumDrive(hwMap);
    }

    public double getCurrent(){
        //method to get the current position for pid calculation
        return drive.getRawExternalHeading();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize motors to be tuned and loop to control them
        init(hardwareMap);

        waitForStart();

        controller.setTarget(0);

        while(opModeIsActive()){
            controller.setCoefficients(kP,kI,kD);
            controller.a = tunerA;

            drive.setWeightedDrivePower(new Pose2d(0,0, controller.calculate(getCurrent())));

            if(timer.seconds() > delay) {
                timer.reset();
                if(controller.target == distance){
                    controller.setTarget(0);
                }
                else{
                    controller.setTarget(distance);
                }
            }

            telemetry.addData("Current position", getCurrent());
            telemetry.addData("Target position", controller.target);
            telemetry.addData("output", controller.calculate(getCurrent()));
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }
}
