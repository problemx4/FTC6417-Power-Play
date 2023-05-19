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

@Config
@TeleOp(name = "PIDTuner", group = "TeleOp")
public class PIDtuner6417 extends LinearOpMode {

    public static double kP = 0.6;
    public static double kI = 0.4;
    public static double kD = 0.4;
    public static double kF = 0.21;
    public static double tunerA = 0.999;

    public static int start = ControlConstants.sliderLowPos;

    public static int distance = ControlConstants.sliderMedPos;

    int target = start;

    private ElapsedTime timer = new ElapsedTime();
    public static double delay = 4.0;

    Hardware6417 robot;

    public void init(HardwareMap hwMap){
        //initialize whatever is to be tuned
        robot = new Hardware6417(hwMap);
    }

    public double getCurrent(){
        //method to get the current position for pid calculation
        return robot.slider.getCurrentPosition();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize motors to be tuned and loop to control them
        init(hardwareMap);

        waitForStart();



        while(opModeIsActive()){
            robot.slidePID.setCoefficients(kP,kI,kD,kF);
            robot.slidePID.a = tunerA;

            robot.autoSlide(target);

            if(timer.seconds() > delay) {
                timer.reset();
                if(target == distance){
                    target = start;
                }
                else{
                    target = distance;
                }
            }

            telemetry.addData("Current position", getCurrent());
            telemetry.addData("Target position", target);
            telemetry.addData("output", robot.slider.getPower());
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }
}
