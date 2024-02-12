package org.firstinspires.ftc.teamcode.TeleOp.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Slow Mo Hold Down Test")

public class _20240210_SlowMo_HoldDown_Test extends LinearOpMode {

    private double Driving_Speed = 0.85;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer Localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            Localizer.update();

            Pose2d RobotPose = Localizer.getPoseEstimate();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double RobotHeading = RobotPose.getHeading();

            if (gamepad1.dpad_up && Driving_Speed != 0.1) {
                Driving_Speed = 0.1;
                backLeftMotor.setPower(-Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(-Driving_Speed);
            } else {
                Driving_Speed = 0.85;
            }

            if (gamepad1.back) {
                Localizer.setPoseEstimate(new Pose2d(RobotPose.getX(), RobotPose.getY(), 0));
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-RobotHeading) - y * Math.sin(-RobotHeading);
            double rotY = x * Math.sin(-RobotHeading) + y * Math.cos(-RobotHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.dpad_up) {
                frontLeftPower = -0.1/Driving_Speed;
                backLeftPower = -0.1/Driving_Speed;
                frontRightPower = -0.1/Driving_Speed;
                backRightPower = -0.1/Driving_Speed;
            }

            frontLeftMotor.setPower(frontLeftPower * Driving_Speed);
            backLeftMotor.setPower(backLeftPower * Driving_Speed);
            frontRightMotor.setPower(frontRightPower * Driving_Speed);
            backRightMotor.setPower(backRightPower * Driving_Speed);

            if (gamepad1.dpad_up) {
                Driving_Speed = 0.1;
                frontLeftPower = 0.1;
                backLeftPower = 0.1;
                frontRightPower = 0.1;
                backRightPower = 0.1;
            } else {
                Driving_Speed = 0.85;
            }

            telemetry.addData("Heading: ", RobotHeading);
            telemetry.addData("frontLeftPower: ", frontLeftPower);
            telemetry.addData("backLeftPower: ", backLeftPower);
            telemetry.addData("frontRightPower: ", frontRightPower);
            telemetry.addData("backRightPower: ", backRightPower);

            telemetry.addLine("");
            telemetry.addLine("Genshin UID: 642041765");
            telemetry.update();
        }
    }
}