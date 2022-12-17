package org.firstinspires.ftc.teamcode.BluCru6417;

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
    public DcMotorEx leftSlider = null;
    public DcMotorEx rightSlider = null;

    public Servo turret        = null;
    public Servo grabber        = null;

    //angle variables
    double lastAngle;
    double globalAngle;

    //camera variables
    double[] subMatCenter = {0.8,0.6}; //NOT coordinates, these values are the % across the screen,.5 being the exact center, x,y from top left
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
        super(ahwMap);
        initIntake(ahwMap);

        // Save reference to Hardware map
        hwMap       = ahwMap;
    }

    /* Initialize standard Hardware interfaces */
    public void initIntake(HardwareMap ahwMap){
        // Define and initialize motor and servo
        leftSlider  = ahwMap.get(DcMotorEx.class, "LeftSlider");
        rightSlider = ahwMap.get(DcMotorEx.class, "RightSlider");

        turret      = ahwMap.get(Servo.class, "Turret");
        grabber     = ahwMap.get(Servo.class, "Grabber");

        //set direction of motors accordingly
        leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        //set all motors to zero power
        leftSlider.setPower(0);
        rightSlider.setPower(0);

        //set brake behavior
        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset sliders
        resetSliders();
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
     * stop() - stops all motors
     */

    public void telemetry(Telemetry tele){
        tele.addData("left front Pos", getWheelTicks(0));
        tele.addData("left rear Pos", getWheelTicks(1));
        tele.addData("right rear Pos", getWheelTicks(2));
        tele.addData("right front Pos", getWheelTicks(3));

        tele.addData("left slider Pos", leftSlider.getCurrentPosition());
        tele.addData("right slider pos", rightSlider.getCurrentPosition());

        tele.addData("Turret position", turret.getPosition());

        tele.addData("Cumulative Angle", Math.toDegrees(getCumulativeAngle()));
        tele.addData("RR Angle", Math.toDegrees(getRawExternalHeading()));

        tele.update();
    }



    public void rrHolonomicDrive(double power, double horizontal, double vertical, double rotation, boolean maintainHeading){
        Vector2d input = new Vector2d(horizontal, vertical);

        input = input.rotated(Math.toRadians(-90));

        if(maintainHeading){
            double heading = getCumulativeAngle();
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
        if(leftSlider.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //check if slider is going beyond limits
        if(limiter){
            if((leftSlider.getCurrentPosition() > sliderMaxPos || rightSlider.getCurrentPosition() > sliderMaxPos ) && power > 0){
                leftSlider.setPower(0);
                rightSlider.setPower(0);
                return;
            }
            if((leftSlider.getCurrentPosition() < 0 || rightSlider.getCurrentPosition() < 0 ) && power < 0){
                leftSlider.setPower(0);
                rightSlider.setPower(0);
                return;
            }
        }

        leftSlider.setPower(power);
        rightSlider.setPower(power);
    }

    public void autoSlide(int position){
        if(leftSlider.getTargetPosition() != position || rightSlider.getTargetPosition() != position){
            leftSlider.setTargetPosition(position);
            rightSlider.setTargetPosition(position);

            if(leftSlider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            leftSlider.setPower(autoSlidePower);
            rightSlider.setPower(autoSlidePower);
        }
    }

    public void clearSliders(int clearDelta){
        int position = ((leftSlider.getCurrentPosition() + rightSlider.getCurrentPosition()) / 2) + clearDelta;

        leftSlider.setTargetPosition(position);
        rightSlider.setTargetPosition(position);

        if(leftSlider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftSlider.setPower(clearSlidePower);
        rightSlider.setPower(clearSlidePower);
    }

    public void resetSliders(){
        //reset motor encoders
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //use motor encoders
        leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motors to run to zero to avoid problems
        leftSlider.setTargetPosition(0);
        rightSlider.setTargetPosition(0);
        //lololol
    }



    //turret code
    public void autoTurret(double position){
        turret.setPosition(position);
    }

    public void manualTurret(double delta){
        double newPos = Range.clip(turret.getPosition() + (delta * manualServoDelta), turretMinPos, turretMaxPos);
        autoTurret(newPos);
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



    public void stop(){
        setWeightedDrivePower(new Pose2d(0,0,0));
        manualSlide(0,true);
    }
}
