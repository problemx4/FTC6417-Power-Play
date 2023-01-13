package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru6417.PIDcontroller6417;

@Config
@TeleOp(name = "PIDTuner", group = "TeleOp")
public class PIDtuner6417 extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double distance = 1000;

    private ElapsedTime timer = new ElapsedTime();
    private double delay = 5.0;

    PIDcontroller6417 controller = new PIDcontroller6417(kP,kI,kD);

    DcMotorEx motor;

    public void init(HardwareMap hwMap){
        //initialize whatever is to be tuned
        motor = hardwareMap.get(DcMotorEx.class, "LeftSlider");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getCurrent(){
        //method to get the current position for pid calculation
        return motor.getCurrentPosition();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initialize motors to be tuned and loop to control them
        init(hardwareMap);

        while(opModeIsActive()){
            controller.setCoefficients(kP,kI,kD);

            motor.setVelocity(controller.calculate(getCurrent()));

            if(!controller.isBusy()){
                timer.reset();
                if(timer.seconds() > delay){
                    if(controller.target == 0){
                        controller.setTarget(distance);
                    }
                    else{
                        controller.setTarget(0);
                    }
                }
            }

            telemetry.addData("Current position", getCurrent());
            telemetry.addData("Target position", controller.target);
            telemetry.update();
        }
    }
}
