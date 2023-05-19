package org.firstinspires.ftc.teamcode.BluCru6417.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "RPM tester", group = "TeleOp")
public class RPMTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        double time = 10.0;
        double lastTime = 0;
        int count = 0;
        List<Double> lastTicks = Arrays.asList(Double.valueOf(0),Double.valueOf(0),Double.valueOf(0),Double.valueOf(0));;
        List<Double> avgRPMs = Arrays.asList(Double.valueOf(0),Double.valueOf(0),Double.valueOf(0),Double.valueOf(0));

        leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "RearLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "RearRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontRight");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<DcMotorEx> motors;

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for(DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        telemetry.addData("Press X to start", "");
        telemetry.update();

        while(opModeIsActive()){
            if(gamepad1.a){
                timer.reset();

                for(int i = 0; i < motors.size(); i++){
                    lastTicks.set(i,Double.valueOf(motors.get(i).getCurrentPosition()));
                    motors.get(i).setPower(1.0);
                }

                while (timer.seconds() <= time){
                    //calculate deltatime
                    double deltaTime = timer.seconds() - lastTime;
                    lastTime = timer.seconds();
                    //calculate RPM for each motor
                    for(int i = 0; i < motors.size(); i++){
                        Double currentTick = Double.valueOf(motors.get(i).getCurrentPosition());
                        double deltaTick = currentTick - lastTicks.get(i);
                        lastTicks.set(i,currentTick);
                        double RPM = deltaTick / deltaTime;
                        telemetry.addData(String.valueOf(i),RPM);
                        avgRPMs.set(i,avgRPMs.get(i) + RPM);
                    }
                    count++;
                    telemetry.addData("Time",timer.seconds());
                    telemetry.update();
                }

                for(int i = 0; i < motors.size(); i++){
                    avgRPMs.set(i, avgRPMs.get(i) / Double.valueOf(count));
                }

                for(DcMotorEx motor: motors){
                    motor.setPower(0.0);
                }

                telemetry.addData("Left Front", avgRPMs.get(0));
                telemetry.addData("Left Rear", avgRPMs.get(1));
                telemetry.addData("Right Rear", avgRPMs.get(2));
                telemetry.addData("Right Front", avgRPMs.get(3));
                telemetry.update();
            }
        }
    }
}
