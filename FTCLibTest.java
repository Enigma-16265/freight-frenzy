package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.example.ftclibexamples.SharedOdometry.PositionTracker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.tejasmehta.OdometryCore.OdometryCore;
import com.tejasmehta.OdometryCore.localization.EncoderPositions;
import com.tejasmehta.OdometryCore.localization.HeadingUnit;
import com.tejasmehta.OdometryCore.localization.OdometryPosition;
import com.tejasmehta.OdometryCore.math.CoreMath;

@TeleOp(name="FTC Lib Test")
@Disabled
public class FTCLibTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    //Drivetrain
    private DcMotor wheelFrontRight;
    private DcMotor wheelFrontLeft;
    private DcMotor wheelRearRight;
    private DcMotor wheelRearLeft;

    //Encoders
    private MotorEx verticalRight;
    private MotorEx verticalLeft;
    private MotorEx horizontal;

    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 4.05512;
    public static final double CENTER_WHEEL_OFFSET = 3.4252;
    public static final double WHEEL_DIAMETER = 1.37795;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 4000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    //Troubleshooting
    private static double RIGHT_VERTICAL = 0; //ticks
    private static double LEFT_VERTICAL = 0;
    private static double A_HORIZONTAL = 0;

    BNO055IMU imu;                // Additional Gyro device
    Orientation angles;


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

        //Enocders
        verticalRight = new MotorEx(hardwareMap, "encoder");
        verticalLeft = new MotorEx(hardwareMap, "light");
        horizontal = new MotorEx(hardwareMap, "wheelRearRight");

        //Set motor directions
        /////////////////////////////////////////////////////////////////////////////////
        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelRearLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelRearRight.setDirection(DcMotor.Direction.REVERSE);

        //Set motor modes
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */

        verticalLeft.setDistancePerPulse(DISTANCE_PER_PULSE);
        verticalRight.setDistancePerPulse(DISTANCE_PER_PULSE);
        horizontal.setDistancePerPulse(DISTANCE_PER_PULSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        odometry = new HolonomicOdometry(
                verticalLeft::getDistance,
                verticalRight::getDistance,
                horizontal::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        odometry.updatePose(PositionTracker.robotPose);

        telemetry.addData("Robot Position at Init: ", PositionTracker.robotPose);
        telemetry.addData("Status", "Darth Freighter is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            //Troubleshooting
            if (gamepad1.left_bumper) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                RIGHT_VERTICAL = RIGHT_VERTICAL-1;
                LEFT_VERTICAL = LEFT_VERTICAL-1;
            }

            if (gamepad1.y) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                A_HORIZONTAL = A_HORIZONTAL-1;
            }

            if (gamepad1.x) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                A_HORIZONTAL = A_HORIZONTAL+1;
            }


            if (gamepad1.right_bumper) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                RIGHT_VERTICAL = RIGHT_VERTICAL+1;
                LEFT_VERTICAL = LEFT_VERTICAL+1;
            }

            /*

            if (gamepad2.y) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                A_HORIZONTAL = A_HORIZONTAL+1;
            }

            if (gamepad2.left_bumper) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                LEFT_VERTICAL = LEFT_VERTICAL-1;
            }

            if (gamepad2.right_bumper) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                LEFT_VERTICAL = LEFT_VERTICAL+1;
            }



            if (gamepad1.x) {//(catapult.getPosition() > 0.8 && gamepad1.left_bumper)
                LEFT_VERTICAL = 0;
                RIGHT_VERTICAL = 0;
                A_HORIZONTAL = 0;
            }


            EncoderPositions encoderPositions = new EncoderPositions(LEFT_VERTICAL, RIGHT_VERTICAL, A_HORIZONTAL);
            OdometryPosition position = OdometryCore.getInstance().getCurrentPosition(encoderPositions);

             */

            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            //Troubleshooting
            /*
            telemetry.addData("Vertical Right", RIGHT_VERTICAL);
            telemetry.addData("Vertical Left", LEFT_VERTICAL);
            telemetry.addData("Horizontal", A_HORIZONTAL);

             */
            telemetry.addData("Vertical Left", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal", horizontal.getCurrentPosition());
            telemetry.addData("Position", PositionTracker.robotPose);
            telemetry.update();

            //Drivetrain
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            double wheelFrontRightPower = pivot - vertical + horizontal;
            double wheelFrontLeftPower = pivot - vertical - horizontal;
            double wheelRearRightPower = -pivot - vertical - horizontal;
            double wheelRearLeftPower = -pivot - vertical + horizontal;

            wheelFrontRightPower = Range.clip(wheelFrontRightPower, -0.75, 0.75);
            wheelFrontLeftPower = Range.clip(wheelFrontLeftPower, -0.75, 0.75);
            wheelRearRightPower = Range.clip(wheelRearRightPower, -0.75, 0.75);
            wheelRearLeftPower = Range.clip(wheelRearLeftPower, -0.75, 0.75);

            wheelFrontRight.setPower(wheelFrontRightPower);
            wheelRearRight.setPower(wheelFrontLeftPower);
            wheelFrontLeft.setPower(wheelRearRightPower);
            wheelRearLeft.setPower(wheelRearLeftPower);


            idle();

        }
    }

}


