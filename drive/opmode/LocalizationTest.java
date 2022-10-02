package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Disabled
public class LocalizationTest extends LinearOpMode {

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 0.688975; // in
    public static double GEAR_RATIO = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "light"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "encoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "wheelRearRight"));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //frontEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.addData("front", frontEncoder.getCurrentPosition());
            telemetry.addData("right inches", encoderTicksToInches(rightEncoder.getCurrentPosition()));
            telemetry.addData("left inches", encoderTicksToInches(leftEncoder.getCurrentPosition()));
            telemetry.addData("front inches", encoderTicksToInches(frontEncoder.getCurrentPosition()));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

}

