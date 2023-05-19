package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BluCru6417.ControlConstants;

@TeleOp (name = "TestServo", group = "TeleOp")
public class ServoTest extends LinearOpMode implements ControlConstants {
    private Servo ARM = null;
    private Servo WRIST = null;
    private Servo GRABBER = null;
    private Servo LEFTRETRACT = null;
    private Servo RIGHTRETRACT = null;
    private Servo TWISTER = null;

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
        LEFTRETRACT = hardwareMap.get(Servo.class,"LeftRetract");
        LEFTRETRACT.setPosition(leftOdoRetractPos);
        RIGHTRETRACT = hardwareMap.get(Servo.class,"RightRetract");
        RIGHTRETRACT.setPosition(rightOdoRetractPos);
        TWISTER = hardwareMap.get(Servo.class,"Twister");
        TWISTER.setPosition(twisterMidPos);

        telemetry.addData("Ready to start.", "");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while(opModeIsActive()){
            double servoControl = -gamepad1.left_stick_y * .05 * runtime.seconds();
            runtime.reset();

            if(gamepad1.x && !lastGX1){
                ServoState = (ServoState + 1) % 6;
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
                    LEFTRETRACT.setPosition(LEFTRETRACT.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "LEFT RETRACT");
                    telemetry.addData("Servo Pos", LEFTRETRACT.getPosition());
                    break;
                case 4:
                    RIGHTRETRACT.setPosition(RIGHTRETRACT.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "RIGHT RETRACT");
                    telemetry.addData("Servo Pos", RIGHTRETRACT.getPosition());
                    break;
                case 5:
                    TWISTER.setPosition(TWISTER.getPosition() + (servoControl));
                    telemetry.addData("Servo:", "TWISTER");
                    telemetry.addData("Servo Pos", TWISTER.getPosition());
                    break;
            }

            telemetry.addData("Time", runtime.time());
            telemetry.update();
        }
    }
}
