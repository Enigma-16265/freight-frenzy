package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

 */

import java.io.PipedOutputStream;
import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "Blue Autonomous")
//@Disabled
public class RemoteAutonomousBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //OpenCvWebcam robotWebcam;
    //OpenCV.SkystoneDeterminationPipeline pipeline;

    private OpenCvCamera robotWebcam;
    private ContourPipeline pipeline;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    //Motors
    //Drivetrain
    private DcMotor wheelFrontRight;
    private DcMotor wheelFrontLeft;
    private DcMotor wheelRearRight;
    private DcMotor wheelRearLeft;

    //Intake and Turntable
    private DcMotor intake;
    private DcMotor turntable;

    //Servos
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo box;
    private Servo catapult;

    BNO055IMU imu;                // Additional Gyro device
    Orientation angles;

    //Servo init values
    private static final double RIGHT_LINKAGE_DOWN = -0.3;
    private static final double RIGHT_LINKAGE_UP = 0.3;

    private static final double LEFT_LINKAGE_DOWN = -0.3;
    private static final double LEFT_LINKAGE_UP = 0.3;

    private static final double BOX_OUT = 0.93;
    private static final double BOX_IN = 0.52;

    private static final double CATAPULT_IN = 0.79;
    private static final double CATAPULT_OUT = 0.265;

    static final double DRIVE_SPEED = 0.45;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.40;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.025;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.007;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() throws InterruptedException {

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        robotWebcam.setPipeline(pipeline);

        // Webcam Streaming
        robotWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotWebcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        //Drivetrain
        wheelFrontRight = hardwareMap.get(DcMotor.class, "wheelFrontRight");
        wheelFrontLeft = hardwareMap.get(DcMotor.class, "wheelFrontLeft");
        wheelRearRight = hardwareMap.get(DcMotor.class, "wheelRearRight");
        wheelRearLeft = hardwareMap.get(DcMotor.class, "wheelRearLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");
        turntable = hardwareMap.get(DcMotor.class, "turntable");

        //Servos
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");
        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        box = hardwareMap.get(Servo.class, "box");
        catapult = hardwareMap.get(Servo.class, "catapult");

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelRearLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelRearRight.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);

        turntable.setDirection(DcMotor.Direction.FORWARD);

        //Set motor modes
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Setting motor mode regarding encoders
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Set servo positions
        rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
        leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
        box.setPosition(BOX_IN);
        catapult.setPosition(CATAPULT_IN);

        telemetry.addData("Mode", "working...");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        double rectangleArea = pipeline.getRectArea();
        double position = pipeline.getRectMidpointX();

        telemetry.addData("Mode", "ready");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        rectangleArea = pipeline.getRectArea();
        position = pipeline.getRectMidpointX();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            rectangleArea = pipeline.getRectArea();
            position = pipeline.getRectMidpointX();
            sleep(2000);

            //TOP
            //RIGHT
            if (position > 400) {

                gyroDrive(0.45, 150, 0);

                gyroTurn(TURN_SPEED, -95);
                gyroHold(TURN_SPEED, -95, 0.5);

                gyroDrive(0.45, 230, -95);
                sleep(200);

                duck(2500, 0.75);

                gyroTurn(TURN_SPEED, -130);
                gyroHold(TURN_SPEED, -130, 0.3);

                gyroDrive(0.45, -1000, -130);
                sleep(200);

                leftLinkage.setPosition(LEFT_LINKAGE_UP);
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                sleep(200);

                catapult.setPosition(CATAPULT_OUT);
                sleep(800);

                box.setPosition(0.95);
                sleep(500);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                sleep(100);

                gyroTurn(TURN_SPEED, -170);
                gyroHold(TURN_SPEED, -170, 0.3);

                gyroDrive(DRIVE_SPEED, 170, -170); //180
                sleep(200);

                gyroTurn(TURN_SPEED, 72); //DECREASE GOES CLOSER TO HUB
                gyroHold(TURN_SPEED, 72, 0.3);

                gyroDrive(DRIVE_SPEED, 700, 72);
                sleep(100);

                gyroIntakeDrive(DRIVE_SPEED, 340, 72, -300, 0.5);

                intakeIntake(-400, 0.5);
                sleep(100);

                box.setPosition(BOX_IN);
                sleep(100);

                Drive(-150, 150, 150, -150, 0.5);
                sleep(100);

                gyroTurn(TURN_SPEED, 130);
                gyroHold(TURN_SPEED, 130, 0.3);

                leftLinkage.setPosition(LEFT_LINKAGE_UP);
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                sleep(200);

                catapult.setPosition(CATAPULT_OUT);
                sleep(800);

                box.setPosition(0.95);
                sleep(500);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                sleep(100);

                gyroTurn(TURN_SPEED, 160);
                gyroHold(TURN_SPEED, 160, 0.3);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 650, 160);
                sleep(100);

                gyroTurn(TURN_SPEED, 90);
                gyroHold(TURN_SPEED, 90, 0.3);

                Drive(-350, 350, 350, -350, 0.5);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 900, 90);
                sleep(100);

                sleep(10000);


                //MIDDLE
            } else if (position > 150 && position < 400) {
                telemetry.addData("Barcode Position", "Center");
                telemetry.update();

                gyroDrive(0.45, 150, 0);

                gyroTurn(TURN_SPEED, -95);
                gyroHold(TURN_SPEED, -95, 0.3);

                gyroDrive(DRIVE_SPEED, 230, -95);
                sleep(200);

                duck(2500, 0.75);

                gyroTurn(TURN_SPEED, -130);
                gyroHold(TURN_SPEED, -130, 0.3);

                gyroDrive(0.4, -820, -130);
                sleep(200);

                catapult.setPosition(0.31);
                sleep(800);

                gyroDrive(DRIVE_SPEED, -100, -130);
                sleep(200);

                box.setPosition(0.95);
                sleep(500);

                gyroDrive(0.45, 100, -130); //180
                sleep(200);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                gyroTurn(TURN_SPEED, -170);
                gyroHold(TURN_SPEED, -170, 0.3);

                gyroDrive(0.45, 170, -170);
                sleep(200);

                gyroTurn(TURN_SPEED, 71);
                gyroHold(TURN_SPEED, 71, 0.3);

                gyroDrive(DRIVE_SPEED, 950, 71);
                sleep(100);

                gyroIntakeDrive(DRIVE_SPEED, 300, 71, -300, 0.5);
                sleep(100);

                intakeIntake(-450, 0.5);
                sleep(100);

                box.setPosition(BOX_IN);
                sleep(100);

                gyroTurn(TURN_SPEED, 120); //decreased
                gyroHold(TURN_SPEED, 120, 0.3);

                gyroDrive(DRIVE_SPEED, -150, 120);
                sleep(100);

                leftLinkage.setPosition(LEFT_LINKAGE_UP);
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                sleep(200);

                catapult.setPosition(CATAPULT_OUT);
                sleep(800);

                box.setPosition(0.95);
                sleep(500);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 150, 120);
                sleep(100);


                gyroTurn(TURN_SPEED, 180);
                gyroHold(TURN_SPEED, 180, 0.3);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 650, 180);
                sleep(100);

                gyroTurn(TURN_SPEED, 90);
                gyroHold(TURN_SPEED, 90, 0.3);

                Drive(-370, 370, 370, -370, 0.5);
                sleep(100);

                gyroDrive(0.5, 1050, 90);
                sleep(100);

                sleep(10000);


                //LEFT
                //BOTTOM
            } else {
                telemetry.addData("Barcode Position", "Left");
                telemetry.update();

                gyroDrive(0.45, 150, 0);

                gyroTurn(TURN_SPEED, -95);
                gyroHold(TURN_SPEED, -95, 0.3);

                gyroDrive(0.45, 230, -95);
                sleep(150);

                duck(2500, 0.75);

                gyroTurn(TURN_SPEED, -127);
                gyroHold(TURN_SPEED, -127, 0.3);

                gyroDrive(0.4, -770, -127);
                sleep(200);

                catapult.setPosition(0.21);
                sleep(800);

                gyroDrive(DRIVE_SPEED, -100, -127);
                sleep(200);

                box.setPosition(BOX_OUT);
                sleep(500);

                gyroDrive(DRIVE_SPEED, 100, -127);
                sleep(100);

                catapult.setPosition(CATAPULT_IN);
                sleep(150);

                gyroTurn(TURN_SPEED, -170);
                gyroHold(TURN_SPEED, -170, 0.3);

                gyroDrive(0.45, 170, -170);
                sleep(200);

                gyroTurn(TURN_SPEED, 70.5); //DECREASE TO IN TOWARDS HUB
                gyroHold(TURN_SPEED, 70.5, 0.3);

                gyroDrive(DRIVE_SPEED, 1470, 70.5);
                sleep(100);

                intakeIntake(-550, 0.5);
                sleep(100);

                box.setPosition(BOX_IN);
                sleep(100);

                gyroDrive(DRIVE_SPEED, -270, 70.5);
                sleep(100);

                gyroTurn(TURN_SPEED, 140);
                gyroHold(TURN_SPEED, 140, 0.3);

                leftLinkage.setPosition(LEFT_LINKAGE_UP);
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                sleep(200);

                catapult.setPosition(CATAPULT_OUT);
                sleep(800);

                box.setPosition(0.95);
                sleep(500);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                sleep(100);

                gyroTurn(TURN_SPEED, 150);
                gyroHold(TURN_SPEED, 150, 0.3);

                gyroDrive(DRIVE_SPEED, 500, 150);
                sleep(100);

                gyroTurn(TURN_SPEED, 90);
                gyroHold(TURN_SPEED, 90, 0.3);

                Drive(-360, 360, 360, -360, 0.5);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 950, 90);
                sleep(100);

                sleep(10000);


            }
        }

    }
            /*
            if (1==1) {

                gyroTurn(TURN_SPEED, 100);
                gyroHold(TURN_SPEED, 100, 0.5);

                gyroDrive(0.35, 100, 100);
                sleep(200);

                duck(-2500, 0.7);

                gyroTurn(TURN_SPEED, 163);
                gyroHold(TURN_SPEED, 163, 0.3);

                gyroDrive(DRIVE_SPEED, -800, 163);
                sleep(200);

                catapult.setPosition(0.83);
                sleep(800);

                gyroDrive(DRIVE_SPEED, -175, 163);
                sleep(200);

                box.setPosition(BOX_OUT);
                sleep(500);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                gyroTurn(TURN_SPEED, -90);
                gyroHold(TURN_SPEED, -90, 0.3);

                gyroDrive(DRIVE_SPEED, 1300, -90);
                sleep(100);

                gyroTurn(TURN_SPEED, -60);
                gyroHold(TURN_SPEED, -60, 0.3);

                Drive(350, -350, -350, 350, 0.5);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 1000, -60);
                sleep(100);

                intakeIntake(-400, 0.55);

                gyroDrive(DRIVE_SPEED, -1040, -57);
                box.setPosition(BOX_IN);
                sleep(100);

                gyroTurn(TURN_SPEED, -115);
                gyroHold(TURN_SPEED, -115, 0.3);

                gyroDrive(DRIVE_SPEED, -550, -115);
                sleep(100);

                catapult.setPosition(0.83);
                sleep(800);

                gyroDrive(DRIVE_SPEED, -135, -115);
                sleep(200);

                box.setPosition(BOX_OUT);
                sleep(500);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                gyroDrive(0.5, 600, -115);
                sleep(100);

                gyroTurn(TURN_SPEED, -60);
                gyroHold(TURN_SPEED, -60, 0.1);

                Drive(320, -320, -320, 320, 0.5);
                sleep(100);

                gyroDrive(0.6, 800, -60);
                sleep(100);

             */

    //TOP
                /*
                gyroTurn(TURN_SPEED, 100);
                gyroHold(TURN_SPEED, 100, 0.5);

                gyroDrive(0.35, 100, 100);
                sleep(200);

                duck(-2500, 0.7);

                gyroTurn(TURN_SPEED, 160);
                gyroHold(TURN_SPEED, 160, 0.3);

                gyroDrive(DRIVE_SPEED, -1100, 160);
                sleep(200);

                leftLinkage.setPosition(LEFT_LINKAGE_UP);
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                sleep(200);

                catapult.setPosition(CATAPULT_OUT);
                sleep(800);

                box.setPosition(BOX_OUT);
                sleep(300);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                sleep(100);

                gyroTurn(TURN_SPEED, -90);
                gyroHold(TURN_SPEED, -90, 0.3);

                gyroDrive(DRIVE_SPEED, 1300, -90);
                sleep(100);

                gyroTurn(TURN_SPEED, -60);
                gyroHold(TURN_SPEED, -60, 0.3);

                Drive(350, -350, -350, 350, 0.5);
                sleep(100);

                gyroDrive(DRIVE_SPEED, 950, -60);
                sleep(100);

                intakeIntake(-400, 0.5);

                //gyroTurn(TURN_SPEED, -60);
                //gyroHold(TURN_SPEED, -60, 0.2);

                gyroDrive(DRIVE_SPEED, -1030, -58);
                box.setPosition(BOX_IN);
                sleep(100);

                gyroTurn(TURN_SPEED, -115);
                gyroHold(TURN_SPEED, -115, 0.3);

                gyroDrive(DRIVE_SPEED, -870, -115);
                sleep(100);

                leftLinkage.setPosition(LEFT_LINKAGE_UP);
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                sleep(200);

                catapult.setPosition(CATAPULT_OUT);
                sleep(800);

                box.setPosition(BOX_OUT);
                sleep(300);

                catapult.setPosition(CATAPULT_IN);
                sleep(300);

                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                sleep(100);

                gyroDrive(0.5, 750, -115);
                sleep(100);

                gyroTurn(TURN_SPEED, -60);
                gyroHold(TURN_SPEED, -60, 0.1);

                Drive(350, -350, -350, 350, 0.5);
                sleep(100);

                gyroDrive(0.6, 700, -60);
                sleep(100);
                 */




    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }

    public void duck (int distance, double speed){

        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turntable.setTargetPosition(distance);

        turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turntable.setPower(speed);


        while (turntable.isBusy() && opModeIsActive()) {
            telemetry.addData("Horizontal Slide", turntable.getCurrentPosition());
            telemetry.update();
        }

        turntable.setPower(0);

        turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    public void intakeIntake (int distance, double speed){

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setTargetPosition(distance);

        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setPower(speed);


        while (intake.isBusy() && opModeIsActive()) {
            telemetry.addData("Horizontal Slide", intake.getCurrentPosition());
            telemetry.update();
        }

        intake.setPower(0);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }


    public void gyroDrive(double speed,
                          int distance,
                          double angle) {
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            wheelFrontRight.setTargetPosition(distance);
            wheelFrontLeft.setTargetPosition(distance);
            wheelRearRight.setTargetPosition(distance);
            wheelRearLeft.setTargetPosition(distance);

            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            wheelFrontRight.setPower(speed);
            wheelFrontLeft.setPower(speed);
            wheelRearRight.setPower(speed);
            wheelRearLeft.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (wheelFrontRight.isBusy() && wheelFrontLeft.isBusy() && wheelRearRight.isBusy() && wheelRearLeft.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                wheelFrontLeft.setPower(leftSpeed);
                wheelRearLeft.setPower(leftSpeed);
                wheelFrontRight.setPower(rightSpeed);
                wheelRearRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            wheelFrontRight.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelRearLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroIntakeDrive(double speed,
                                int distance,
                                double angle, int intakeDistance, double intakeSpeed) {
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheelRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            wheelFrontRight.setTargetPosition(distance);
            wheelFrontLeft.setTargetPosition(distance);
            wheelRearRight.setTargetPosition(distance);
            wheelRearLeft.setTargetPosition(distance);

            intake.setTargetPosition(intakeDistance);

            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            wheelFrontRight.setPower(speed);
            wheelFrontLeft.setPower(speed);
            wheelRearRight.setPower(speed);
            wheelRearLeft.setPower(speed);

            intake.setPower(intakeSpeed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (wheelFrontRight.isBusy() && wheelFrontLeft.isBusy() && wheelRearRight.isBusy() && wheelRearLeft.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                wheelFrontLeft.setPower(leftSpeed);
                wheelRearLeft.setPower(leftSpeed);
                wheelFrontRight.setPower(rightSpeed);
                wheelRearRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            wheelFrontRight.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelRearLeft.setPower(0);

            intake.setPower(0);

            // Turn off RUN_TO_POSITION
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void Drive (int frontRightDistance, int frontLeftDistance, int rearRightDistance, int rearLeftDistance, double speed){


        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelFrontRight.setTargetPosition(frontRightDistance);
        wheelFrontLeft.setTargetPosition(frontLeftDistance);
        wheelRearRight.setTargetPosition(rearRightDistance);
        wheelRearLeft.setTargetPosition(rearLeftDistance);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFrontRight.setPower(speed);
        wheelFrontLeft.setPower(speed);
        wheelRearRight.setPower(speed);
        wheelRearLeft.setPower(speed);


        while (wheelFrontRight.isBusy() && wheelFrontLeft.isBusy() && wheelRearRight.isBusy() && wheelRearLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("FrontRightPosition", wheelFrontRight.getCurrentPosition());
            telemetry.addData("FrontLeftPosition", wheelFrontLeft.getCurrentPosition());
            telemetry.update();
        }

        wheelFrontRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelRearRight.setPower(0);
        wheelRearLeft.setPower(0);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {

        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        wheelFrontLeft.setPower(leftSpeed);
        wheelRearLeft.setPower(leftSpeed);
        wheelFrontRight.setPower(rightSpeed);
        wheelRearRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Error", error);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // calculate error in -179 to +180 range  (
        robotError = angles.firstAngle - targetAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}

