package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.tejasmehta.OdometryCore.OdometryCore;
import com.tejasmehta.OdometryCore.localization.EncoderPositions;
import com.tejasmehta.OdometryCore.localization.OdometryPosition;

import java.util.Base64;

@TeleOp(name="TeleOp")
//@Disabled
public class FrieghtFrenzyTeleOp extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Motors
    //Drivetrain
    private DcMotor wheelFrontRight;
    private DcMotor wheelFrontLeft;
    private DcMotor wheelRearRight;
    private DcMotor wheelRearLeft;

    //Intake and Turntable
    private DcMotor intake;
    private DcMotor turntable;
    private DcMotor light;
    private DcMotor encoder;

    //Encoders
    private DcMotor verticalRight;
    private DcMotor verticalLeft;
    private DcMotor horizontal;

    //Servos
    private Servo rightLinkage;
    private Servo leftLinkage;
    private Servo box;
    private Servo catapult;
    private Servo cap;

    private ColorSensor color;


    BNO055IMU imu;                // Additional Gyro device
    Orientation angles;

    //Servo init values
    private static final double RIGHT_LINKAGE_DOWN = -0.3;
    private static final double RIGHT_LINKAGE_UP = 0.3;

    private static final double LEFT_LINKAGE_DOWN = -0.3;
    private static final double LEFT_LINKAGE_UP = 0.3;

    private static final double BOX_OUT = 0.95;
    private static final double BOX_IN = 0.52;
    private static final double BOX_IN_IN = 0.45;

    private static final double CATAPULT_IN = 0.79; //0.749 //0.9
    private static final double CATAPULT_OUT = 0.265; //0.24 for new cap

    private static final double LIGHT_ON = 0.20;

    private static final double CPR = 1000;
    private static final double WHEEL_DIAMETER = 1.37795;
    private static final double LEFT_OFFSET = 2.16535;
    private static final double RIGHT_OFFSET = 1.85039;
    private static final double BACK_OFFSET = 1.53543;

    //String verticalRightName = "wheelFrontRight", verticalLeftName = "wheelFrontLeft", horizontalName = "wheelRearRight";

    //0.96
    //0


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        //Motors
        wheelFrontRight = hardwareMap.get(DcMotor.class, "wheelFrontRight");
        wheelFrontLeft = hardwareMap.get(DcMotor.class, "wheelFrontLeft");
        wheelRearRight = hardwareMap.get(DcMotor.class, "wheelRearRight");
        wheelRearLeft = hardwareMap.get(DcMotor.class, "wheelRearLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");
        turntable = hardwareMap.get(DcMotor.class, "turntable");
        light = hardwareMap.get(DcMotor.class, "light");

        //Enocders
        verticalRight = hardwareMap.get(DcMotor.class, "encoder");
        verticalLeft = hardwareMap.get(DcMotor.class, "light");
        horizontal = hardwareMap.get(DcMotor.class, "wheelRearRight");

        //Servos
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");
        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        box = hardwareMap.get(Servo.class, "box");
        catapult = hardwareMap.get(Servo.class, "catapult");
        cap = hardwareMap.get(Servo.class, "cap");

        color = hardwareMap.get(ColorSensor.class, "color");

        //Set motor directions
        /////////////////////////////////////////////////////////////////////////////////
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

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        OdometryCore.initialize(CPR, WHEEL_DIAMETER, LEFT_OFFSET, RIGHT_OFFSET, BACK_OFFSET);

        //Set servo positionS
        //leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
        //rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
        //box.setPosition(BOX_OUT);
        //catapult.setPosition(CATAPULT_IN);
        color.enableLed(true);

        telemetry.addData("Status", "Darth Freighter is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            EncoderPositions encoderPositions = new EncoderPositions(verticalLeft.getCurrentPosition(), verticalRight.getCurrentPosition(), horizontal.getCurrentPosition());
            OdometryPosition position = OdometryCore.getInstance().getCurrentPosition(encoderPositions);


            */
            telemetry.addData("Vertical Left Position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            /*
            telemetry.addData("Y", position.getY());
            telemetry.addData("X", position.getX());
            telemetry.addData("Heading", position.getHeadingDegrees());

             */
            telemetry.update();



            //TANK DRIVE
            /*
            // drivePower is the power for forward/backward movement
            // rotatePower is the power for rotating the robot
            float drivePower = -gamepad1.left_stick_y;
            float rotatePower = gamepad1.right_stick_x;

            // Flip these signs if the robot rotates the wrong way
            wheelFrontLeft.setPower(Range.clip(drivePower - rotatePower, -0.7, 0.7));
            wheelFrontRight.setPower(Range.clip(drivePower + rotatePower, -0.7, 0.7));
            wheelRearLeft.setPower(Range.clip(drivePower - rotatePower, -0.7, 0.7));
            wheelRearRight.setPower(Range.clip(drivePower + rotatePower, -0.7, 0.7));
             */



            if (color.argb() < 0) {
                light.setPower(0.4);
            } else {
                light.setPower(0);
            }

            //Drivetrain
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            double wheelFrontRightPower = pivot - vertical + horizontal;
            double wheelFrontLeftPower = pivot - vertical - horizontal;
            double wheelRearRightPower = -pivot - vertical - horizontal;
            double wheelRearLeftPower = -pivot - vertical + horizontal;

            if (gamepad1.left_bumper) {
                wheelFrontRightPower = Range.clip(wheelFrontRightPower, -0.3, 0.3);
                wheelFrontLeftPower = Range.clip(wheelFrontLeftPower, -0.3, 0.3);
                wheelRearRightPower = Range.clip(wheelRearRightPower, -0.3, 0.3);
                wheelRearLeftPower = Range.clip(wheelRearLeftPower, -0.3, 0.3);
            } else {
                wheelFrontRightPower = Range.clip(wheelFrontRightPower, -0.8, 0.8);
                wheelFrontLeftPower = Range.clip(wheelFrontLeftPower, -0.8, 0.8);
                wheelRearRightPower = Range.clip(wheelRearRightPower, -0.8, 0.8);
                wheelRearLeftPower = Range.clip(wheelRearLeftPower, -0.8, 0.8);
            }


            wheelFrontRight.setPower(wheelFrontRightPower);
            wheelRearRight.setPower(wheelFrontLeftPower);
            wheelFrontLeft.setPower(wheelRearRightPower);
            wheelRearLeft.setPower(wheelRearLeftPower);


            //Intake & Turntable
            double intakePower = gamepad2.right_stick_y;
            double turntablePower = gamepad2.left_stick_y;

            intakePower = Range.clip(intakePower, -0.5, 0.5);
            turntablePower = Range.clip(turntablePower, -0.7, 0.7);

            intake.setPower(intakePower);
            turntable.setPower(turntablePower);

            //cap
            if (gamepad1.right_bumper) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                cap.setPosition(0.8);
            } else if (gamepad1.x) {
                cap.setPosition(0.1);
            } else {
                cap.setPosition(0.2);
            }

            //Box
            if (gamepad2.y) {
                box.setPosition(BOX_IN);
            } else if (gamepad2.a || gamepad2.x || gamepad2.b) {
                box.setPosition(BOX_IN);
            } else {
                box.setPosition(BOX_OUT);
            }


            //Catapult
            if (gamepad2.left_bumper) {
                catapult.setPosition(CATAPULT_OUT);
            } else if (gamepad2.a) {
                catapult.setPosition(0.192);
                box.setPosition(BOX_IN);
            }  else if (gamepad2.b) {
                catapult.setPosition(0.29);
                box.setPosition(BOX_IN);
            } else {
                catapult.setPosition(CATAPULT_IN);
            }


            //Linkages
            if (gamepad2.right_bumper) {
                rightLinkage.setPosition(RIGHT_LINKAGE_UP);
                leftLinkage.setPosition(LEFT_LINKAGE_UP);
            } else if (gamepad2.x) {
                rightLinkage.setPosition(0.5);
                leftLinkage.setPosition(0.5);
                box.setPosition(BOX_IN);
            } else {
                rightLinkage.setPosition(RIGHT_LINKAGE_DOWN);
                leftLinkage.setPosition(LEFT_LINKAGE_DOWN);
            }


            idle();

        }
    }

}
