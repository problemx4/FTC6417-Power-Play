package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are uppercase
 *
 * Motor channel:   leftFront drive motor:          "FrontLeft" *
 * Motor channel:   rightFront drive motor:         "FrontRight" *
 * Motor channel:   leftRear drive motor:           "RearLeft" *
 * Motor channel:   rightRear drive motor:          "RearRight" *
 * Motor channel:   left slider motor:              "LeftSlider" *
 * Motor channel:   right slider motor:             "RightSlider" *
 * Servo channel:   turret servo:                   "Turret" *
 * Servo channel:   grabber Servo:                  "Grabber" *
 */

public class Hardware6417 extends SampleMecanumDrive implements ControlConstants{

    //TeleOp variables
    public DcMotorEx slider     = null;
    public DcMotorEx auxSlider  = null;

    public Servo turret         = null;
    public Servo grabber        = null;
    public Servo wrist          = null;

    public Servo parallelRetract = null;

    //angle variables
    double lastAngle;
    public double globalAngle;

    //power variables
    public double autoSlidePower;

    //camera variables
    double[] subMatCenter = {0.32,0.5}; //NOT coordinates, these values are the % across the screen,.5 being the exact center, x,y from top left
    int subMatWidth = 80;
    int subMatHeight = 100;

    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 360;

    public OpenCvWebcam webcam;
    public SignalDetectorPipeline pipeline;

    //hardwareMap
    HardwareMap hwMap;

    /* Constructor */
    public Hardware6417(HardwareMap ahwMap){
        //initialize drive train
        super(ahwMap);
        //initialize intake
        initIntake(ahwMap);
        //initialize odo pod retractor
        parallelRetract = ahwMap.get(Servo.class, "ParallelRetractor");
        // Save reference to Hardware map
        hwMap       = ahwMap;
    }

    /* Initialize standard Hardware interfaces */
    public void initIntake(HardwareMap ahwMap){
        // Define and initialize motor and servo
        slider  = ahwMap.get(DcMotorEx.class, "Slider");
        auxSlider = ahwMap.get(DcMotorEx.class, "AuxSlider");

        turret      = ahwMap.get(Servo.class, "Turret");
        grabber     = ahwMap.get(Servo.class, "Grabber");
        wrist       = ahwMap.get(Servo.class, "Wrist");

        //set direction of motors accordingly
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        auxSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        //set all motors to zero power
        slider.setPower(0);
        auxSlider.setPower(0);

        //set brake behavior
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        auxSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //reset sliders
        resetSliders();

        //init autoSlide Variable

        autoSlidePower = highSlidePower;
    }

    public void initCamera(HardwareMap ahwMap, Telemetry tele){
        //initialize camera
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        WebcamName webcamName = ahwMap.get(WebcamName.class, "webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SignalDetectorPipeline(subMatCenter[0], subMatCenter[1],subMatWidth,subMatHeight, CAMERA_WIDTH, CAMERA_HEIGHT);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                tele.addData("Camera Failed","");
                tele.update();
            }
        }); //done initializing camera
    }

    /**
     * Robot Methods:
     * telemetry(tele) - telemetry
     *
     * holonomicDrive(power, angle) - holonomic drive
     * getCumulativeAngle() - get cumulative angle of robot
     * resetAngle() - resets angle
     *
     * rotationPID(double angle) - controls rotation in PID fashion to turn to a specific angle useing IMU
     * translationPID(double position) - gets current position relative to poles using senors than PID's to the position to be in the middle
     *
     * manualSlide(power) - moves sliders
     * autoSlide(position) - runs slider to position
     * resetSliders() - reset slider encoders
     *
     * autoTurret(position) - moves turret to position
     * manualTurret(delta) - manually move turret
     *
     * grab(pos) - sets grabber servo
     * openGrabber() - opens grabber
     * closeGrabber() - closes grabber
     *
     * dropOdo() - drops odo
     * retractOdo() - retracts odo
     *
     * stop() - stops all motors
     */

    public void telemetry(Telemetry tele){
        tele.addData("left front Pos", getWheelTicks(0));
        tele.addData("left rear Pos", getWheelTicks(1));
        tele.addData("right rear Pos", getWheelTicks(2));
        tele.addData("right front Pos", getWheelTicks(3));

        tele.addData("Slider pos", slider.getCurrentPosition());
        tele.addData("auxSlider pos", auxSlider.getCurrentPosition());

        tele.addData("Turret position", turret.getPosition());
        tele.addData("Wrist position", wrist.getPosition());

        tele.addData("Cumulative Angle", Math.toDegrees(getCumulativeAngle()));

        tele.update();
    }



    public void rrHolonomicDrive(double power, double horizontal, double vertical, double rotation, boolean maintainHeading, double heading){
        Vector2d input = new Vector2d(horizontal, vertical);

        input = input.rotated(Math.toRadians(-90));

        if(maintainHeading){
            input = input.rotated(-heading);
        }

        setWeightedDrivePower(new Pose2d(input.getX() * power,input.getY() * power,-rotation * power));
    }

    public double getCumulativeAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        double angles = getRawExternalHeading();

        double deltaAngle = angles - lastAngle;

        if (deltaAngle < -Math.PI)
            deltaAngle += 2 * Math.PI;
        else if (deltaAngle > Math.PI)
            deltaAngle -= 2 * Math.PI;

        globalAngle += deltaAngle;

        lastAngle = angles;

        return globalAngle;
    }

    public void resetAngle(){
        lastAngle = getRawExternalHeading();

        globalAngle = 0;
    }


    public void rotationPID(double angle){

    }

    public void translationPID(double angle){

    }


    public void manualSlide(double power, boolean limiter){
        if(slider.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            auxSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //check if slider is going beyond limits
        if(limiter){
            if((slider.getCurrentPosition() > sliderMaxPos ) && power > 0){
                slider.setPower(0);
                auxSlider.setPower(0);
                return;
            }
            if((slider.getCurrentPosition() < sliderMinPos ) && power < 0){
                slider.setPower(0);
                auxSlider.setPower(0);
                return;
            }
        }

        slider.setPower(power);
        auxSlider.setPower(power);
    }

    public void autoSlide(int position){
        if(slider.getTargetPosition() != position){
            slider.setTargetPosition(position);
            auxSlider.setTargetPosition(position);

            if(slider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                auxSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            slider.setPower(autoSlidePower);
            auxSlider.setPower(autoSlidePower);
        }
    }

    public void clearSliders(int clearDelta){
        int position = (slider.getCurrentPosition()) + clearDelta;

        slider.setTargetPosition(position);
        auxSlider.setTargetPosition(position);

        if(slider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            auxSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        slider.setPower(clearSlidePower);
        auxSlider.setPower(clearSlidePower);
    }

    public void resetSliders(){
        //reset motor encoders
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //use motor encoders
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        auxSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to zero
        slider.setPower(0);
        auxSlider.setPower(0);

        //set motors to run to zero to avoid problems
        slider.setTargetPosition(0);
        auxSlider.setTargetPosition(0);
        //lololol
    }

    public void setAutoSlidePower(double power){
        autoSlidePower = power;
    }



    //turret code
    public double turretAngleToPower(double angle){
        double ratio = angle / (Math.PI / 2.0);
        double quarterDistance = turretLeftPos - turretForwardPos;
        return (quarterDistance * ratio) + turretForwardPos;
    }

    public void autoTurret(double pos){
        if(pos != turret.getPosition()){
            turret.setPosition(pos);
        }
    }

    public void manualTurret(double delta){
        double newPos = Range.clip(turret.getPosition() + (delta * manualServoDelta), turretMinPos, turretMaxPos);
        turret.setPosition(newPos);
    }



    //wrist control
    public void moveWrist(double pos){
        if(pos != wrist.getPosition()){
            wrist.setPosition(pos);
        }
    }

    public void manualMoveWrist(double delta){
        double newPos = Range.clip(wrist.getPosition() + (delta * manualServoDelta), turretMinPos, turretMaxPos);
        wrist.setPosition(newPos);
    }



    public void grab(double pos){
        grabber.setPosition(pos);
    }

    public void openGrabber(){
        grab(grabberOpenPos);
    }

    public void closeGrabber(){
        grab(grabberClosePos);
    }

    public void dropOdo(){
        parallelRetract.setPosition(odoDropPos);
    }

    public void retractOdo(){
        parallelRetract.setPosition(odoRetractPos);
    }

    public void stop(){
        setWeightedDrivePower(new Pose2d(0,0,0));
        manualSlide(0,true);
    }
}
