package org.firstinspires.ftc.teamcode.BluCru6417;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
 * Motor channel:   leftFront drive motor:          "FrontLeft"
 * Motor channel:   rightFront drive motor:         "FrontRight"
 * Motor channel:   leftRear drive motor:           "RearLeft"
 * Motor channel:   rightRear drive motor:          "RearRight"
 * Motor channel:   left slider motor:              "LeftSlider" *
 * Motor channel:   right slider motor:             "RightSlider" *
 * Motor channel:   arm motor:                      "Arm" *
 * Servo channel:   wrist servo:                    "Wrist" *
 * Servo channel:   grabber Servo:                  "Grabber"
 */

public class Hardware6417 extends SampleMecanumDrive implements ControlConstants{

    //TeleOp variables
    /* public op mode members */
    /*
    public DcMotorEx leftFront  = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear   = null;
    public DcMotorEx rightRear  = null;
     */
    public DcMotorEx leftSlider = null;
    public DcMotorEx rightSlider = null;

    public DcMotorEx arm        = null;
    public Servo wrist          = null;

    public Servo grabber        = null;

    //imu variables
    public BNO055IMU imu;

    double lastAngle;
    double globalAngle;

    //camera variables
    double[] subMatCenter = {0.45,0.6}; //NOT coordinates, these values are the % across the screen,.5 being the exact center
    int subMatWidth = 80;
    int subMatHeight = 100;

    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 360;

    public OpenCvWebcam webcam;
    public SignalDetectorPipeline pipeline;

    //hardwareMap
    HardwareMap hwMap = null;

    /* Constructor */
    public Hardware6417(HardwareMap ahwMap){
        super(ahwMap);
        //initIMU(ahwMap);
        initIntake(ahwMap);
        //initDrive(ahwMap);

        // Save reference to Hardware map
        hwMap       = ahwMap;
    }

    /* Initialize standard Hardware interfaces */
    /*
    public void initIMU(HardwareMap ahwMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }
     */

    public void initIntake(HardwareMap ahwMap){
        // Define and initialize motor and servo
        leftSlider  = ahwMap.get(DcMotorEx.class, "LeftSlider");
        rightSlider = ahwMap.get(DcMotorEx.class, "RightSlider");
        arm         = ahwMap.get(DcMotorEx.class, "Arm");

        grabber     = ahwMap.get(Servo.class, "Grabber");
        wrist       = ahwMap.get(Servo.class, "Wrist");

        //set direction of motors accordingly
        leftSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        //set all motors to zero power
        leftSlider.setPower(0);
        rightSlider.setPower(0);
        arm.setPower(0);

        //set brake behavior
        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset sliders
        resetSliders();

        //reset Arm (Maybe not? dont do this if auto ends outside of proper 0 position or if auto starts outside of proper 0 position)
        resetArm();
    }

    /*
    public void initDrive(HardwareMap ahwMap){
        // Define and initialize motor and servo
        leftFront   = hwMap.get(DcMotorEx.class, "FrontLeft");
        leftRear    = hwMap.get(DcMotorEx.class, "BackLeft");
        rightFront  = hwMap.get(DcMotorEx.class, "FrontRight");
        rightRear   = hwMap.get(DcMotorEx.class, "BackRight");

        // Set motor and servo directions based on orientation of motors on robot
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set target positions first so that error doesn't occur
        rightFront.setTargetPosition(0);
        leftFront.setTargetPosition(0);
        rightRear.setTargetPosition(0);
        leftRear.setTargetPosition(0);
    }
     */

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
     * start(startGrabPos) - sets grabber to starting position, other necessary starting conditions
     * telemetry(tele) - telemetry
     *
     * holonomicDrive(power, angle) - holonomic drive
     * getCumulativeAngle() - get cumulative angle of robot
     * resetAngle() - resets angle
     *
     * manualSlide(power) - moves sliders
     * autoSlide(position) - runs slider to position
     * resetSliders() - reset slider encoders
     *
     * manualArm(power) - moves arm
     * autoArm(position) - runs arm to position
     * maintainArm() - sets arm power based on cos(getArmAngle()) to maintain its position (may not be necessary)
     * getArmAngle() - returns the angle of the arm based on encoder
     * resetArm() - reset arm encoder
     *
     * manualWrist() - moves wrist
     * autoWrist() - uses wristAngleToPower(getArmAngle()) to set wrist position automatically
     * wristAngleToPower(angle) - converts desired radian angle into servo power
     *
     * grab(pos) - sets grabber servo
     * openGrabber() - opens grabber
     * closeGrabber() - closes grabber
     *
     * stop() - stops all drive motors
     * end() - stops all motors
     */

    public void start(){
        stop();
        grabber.setPosition(grabberClosePos);
    }

    public void telemetry(Telemetry tele){
        tele.addData("left front Pos", getWheelTicks().get(0));
        tele.addData("left rear Pos", getWheelTicks().get(1));
        tele.addData("right rear Pos", getWheelTicks().get(2));
        tele.addData("right front Pos", getWheelTicks().get(3));

        tele.addData("left slider Pos", leftSlider.getCurrentPosition());
        tele.addData("right slider pos", rightSlider.getCurrentPosition());
        tele.addData("arm pos", arm.getCurrentPosition());
        tele.addData("arm angle", getArmAngle());

        tele.addData("Wrist position", wrist.getPosition());

        tele.addData("Cumulative Angle", Math.toDegrees(getCumulativeAngle()));
        tele.addData("RR Angle", Math.toDegrees(getRawExternalHeading()));

        tele.update();
    }


    /*
    public void setDriveSpeeds(double power, double forwardleft, double forwardright, double strafeleft, double straferight) {


        double leftFrontSpeed = forwardleft + strafeleft;
        double rightFrontSpeed = forwardright - straferight;
        double leftRearSpeed = forwardleft - strafeleft;
        double rightRearSpeed = forwardright + straferight;

        double largest = 1.0;
        largest = Math.max(largest, Math.abs(leftFrontSpeed));
        largest = Math.max(largest, Math.abs(rightFrontSpeed));
        largest = Math.max(largest, Math.abs(leftRearSpeed));
        largest = Math.max(largest, Math.abs(rightRearSpeed));

        leftFront.setPower(power * leftFrontSpeed / largest);
        rightFront.setPower(power * rightFrontSpeed / largest);
        leftRear.setPower(power * leftRearSpeed / largest);
        rightRear.setPower(power * rightRearSpeed / largest);
    }

    public void bcHolonomicDrive(double power, double horizontal, double vertical, double rotation, boolean maintainHeading){
        double magnitude = Math.sqrt((Math.pow(horizontal,2.0)) + (Math.pow(vertical,2.0)));
        double inputAngle = Math.atan2(vertical,horizontal);

        double resultAngle = inputAngle;

        if(maintainHeading){
            double robotAngle = getAngle();
            resultAngle = inputAngle - robotAngle;
        }

        double resultVertical = magnitude * Math.sin(resultAngle);
        double resultHorizontal = magnitude * Math.cos(resultAngle);

        setDriveSpeeds(power, resultVertical + rotation, resultVertical - rotation, resultHorizontal, resultHorizontal);
    }
     */

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



    public void manualSlide(double power){
        if(leftSlider.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //check if slider is going beyond limits
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

        leftSlider.setPower(power);
        rightSlider.setPower(power);
    }

    public void autoSlide(int position){
        if(leftSlider.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        leftSlider.setTargetPosition(position);
        rightSlider.setTargetPosition(position);
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
    }



    public void manualArm(double power){
        if(arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        arm.setPower(power);
    }

    public void autoArm(int position){
        if(arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        arm.setTargetPosition(position);
    }

    public void maintainArm(){
        manualArm(Math.sin(getArmAngle()) * armMaintenanceCoeff);
    }

    public double getArmAngle(){
        double armPosition = arm.getCurrentPosition();

        double ratio = armPosition / armVerticalPosition;
        return ratio * (Math.PI / 2.0);
    }

    public void resetArm(){
        //reset motor encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //use motor encoders
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motors to run to zero to avoid problems
        arm.setTargetPosition(0);
    }



    public void manualWrist(double delta){
        double newPosition = Range.clip(wrist.getPosition() + delta,0,1);
        wrist.setPosition(newPosition);
    }

    public void autoWrist(){
        double wristAngle = -getArmAngle();

        wrist.setPosition(wristAngleToPower(wristAngle));
    }

    public static double wristAngleToPower(double angle){
        double ratio = angle / Math.PI;
        return ratio * armVerticalPosition;
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
    }

    public void end(){
        stop();
        manualSlide(0);
        manualArm(0);
    }
}
