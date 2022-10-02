package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.tejasmehta.OdometryCore.OdometryCore;
import com.tejasmehta.OdometryCore.localization.EncoderPositions;
import com.tejasmehta.OdometryCore.localization.HeadingUnit;
import com.tejasmehta.OdometryCore.localization.OdometryPosition;
import com.tejasmehta.OdometryCore.math.CoreMath;

@TeleOp(name="Odometry Core")
@Disabled
public class OdometryCoreTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    //Drivetrain
    private DcMotor wheelFrontRight;
    private DcMotor wheelFrontLeft;
    private DcMotor wheelRearRight;
    private DcMotor wheelRearLeft;

    //Encoders
    private DcMotor verticalRight;
    private DcMotor verticalLeft;
    private DcMotor horizontal;

    final double CPR = 1400;
    final double WHEEL_DIAMETER = 1.5; //1.37795
    final double LEFT_OFFSET = 3.00; //2.16535
    final double RIGHT_OFFSET = 3.00; //1.85039
    final double BACK_OFFSET = 1.00; //1.53543
    final double TICKS_TO_INCH = (Math.PI * WHEEL_DIAMETER) / CPR;

    //Troubleshooting
    /*
    private static double RIGHT_VERTICAL = 0; //ticks
    private static double LEFT_VERTICAL = 0;
    private static double A_HORIZONTAL = 0;
     */

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
        verticalRight = hardwareMap.get(DcMotor.class, "encoder");
        verticalLeft = hardwareMap.get(DcMotor.class, "light");
        horizontal = hardwareMap.get(DcMotor.class, "wheelRearRight");

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

        verticalLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        OdometryCore.initialize(CPR, WHEEL_DIAMETER, LEFT_OFFSET, RIGHT_OFFSET, BACK_OFFSET);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Darth Freighter is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Troubleshooting
            /*
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

 */

            OdometryPosition p = OdometryCore.getInstance().getCurrentPosition(new EncoderPositions(-verticalLeft.getCurrentPosition(), verticalRight.getCurrentPosition(), horizontal.getCurrentPosition()));


            telemetry.addData("Vertical Left", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal", horizontal.getCurrentPosition());
            telemetry.addData("Y", p.getY());
            telemetry.addData("X", p.getX());
            telemetry.addData("Heading", p.getHeadingDegrees());
            telemetry.update();

            /*
            telemetry.addData(
                    "ODOM_POS",
                    "Odometry Position",
                    ":",
                    " X: " + p.getX() + ", Y: " + p.getY() + ", Heading: " + p.getHeadingDegrees()
            );
            telemetry.addData("ODOM_L",
                    "Left Odometry Wheel",
                    ":",
                    " Ticks: " + -verticalLeft.getCurrentPosition() + " Inches: " + ticksToInches(-verticalLeft.getCurrentPosition())
            );
            telemetry.addData("ODOM_R",
                    "Right Odometry Wheel",
                    ":",
                    " Ticks: " + verticalRight.getCurrentPosition() + " Inches: " + ticksToInches(verticalRight.getCurrentPosition())
            );
            telemetry.addData("ODOM_B",
                    "Back Odometry Wheel",
                    ":",
                    " Ticks: " + horizontal.getCurrentPosition() + " Inches: " + ticksToInches(horizontal.getCurrentPosition())
            );

             */

            telemetry.update();

        /*
        EncoderPositions encoderPositions = new EncoderPositions(LEFT_VERTICAL, RIGHT_VERTICAL, A_HORIZONTAL);
        OdometryPosition position = OdometryCore.getInstance().getCurrentPosition(encoderPositions);

        //Troubleshooting
        telemetry.addData("Vertical Right", RIGHT_VERTICAL);
        telemetry.addData("Vertical Left", LEFT_VERTICAL);
        telemetry.addData("Horizontal", A_HORIZONTAL);
        telemetry.addData("Vertical Left", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right", verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal", -horizontal.getCurrentPosition());
        telemetry.addData("Y", position.getY());
        telemetry.addData("X", position.getX());
        telemetry.addData("Heading", position.getHeadingDegrees());
        telemetry.update();
         */

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

    double ticksToInches(double ticks) {
        return ticks * TICKS_TO_INCH;
    }

}


