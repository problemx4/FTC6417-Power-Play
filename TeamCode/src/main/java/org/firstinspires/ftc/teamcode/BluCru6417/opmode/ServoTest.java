package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;

import java.util.List;

@TeleOp (name = "TestServo", group = "TeleOp")
public class ServoTest extends LinearOpMode implements ControlConstants {
    private Servo ARM = null;
    private Servo WRIST = null;
    private Servo GRABBER = null;
    private Servo PARALLEL = null;

    int ServoState = 0;
    boolean lastGX1 = false;

    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        //Put name of motor you want to test in deviceName
        ARM = hardwareMap.get(Servo.class,"Turret");
        ARM.setPosition(turretForwardPos);
        WRIST = hardwareMap.get(Servo.class,"Wrist");
        WRIST.setPosition(retractWristPos);
        GRABBER = hardwareMap.get(Servo.class,"Grabber");
        GRABBER.setPosition(grabberClosePos);
        PARALLEL = hardwareMap.get(Servo.class,"ParallelRetractor");
        PARALLEL.setPosition(odoRetractPos);

        telemetry.addData("Ready to start.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()){
            double servoControl = -gamepad1.left_stick_y * .001;

            if(gamepad1.x && !lastGX1){
                ServoState = (ServoState + 1) % 4;
            }
            lastGX1 = gamepad1.x;


            switch(ServoState){
                case 0:
                    ARM.setPosition(ARM.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "ARM");
                    telemetry.addData("Servo Pos", ARM.getPosition());
                    break;
                case 1:
                    WRIST.setPosition(WRIST.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "WRIST");
                    telemetry.addData("Servo Pos", WRIST.getPosition());
                    break;
                case 2:
                    GRABBER.setPosition(GRABBER.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "GRABBER");
                    telemetry.addData("Servo Pos", GRABBER.getPosition());
                    break;
                case 3:
                    PARALLEL.setPosition(PARALLEL.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "PARALLEL");
                    telemetry.addData("Servo Pos", PARALLEL.getPosition());
                    break;
            }

            telemetry.addData("Time", runtime.time());
            telemetry.update();
        }
    }
}
