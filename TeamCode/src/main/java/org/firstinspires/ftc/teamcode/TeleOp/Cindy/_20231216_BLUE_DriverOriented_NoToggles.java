package org.firstinspires.ftc.teamcode.TeleOp.Cindy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.DroneLauncherInstance;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "0 BLUE Driver Oriented - No Toggles")

public class _20231216_BLUE_DriverOriented_NoToggles extends LinearOpMode {

    private int Arm_Adjustment_Value = 50;

    private double Driving_Speed = 0.85;
    double armSpeed = 0.15;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer Localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

        ArmInstance Arm = new ArmInstance();
        ClawInstance Claw = new ClawInstance();
        DroneLauncherInstance DroneLauncher = new DroneLauncherInstance();

        Arm.initializeArm(hardwareMap);
        Claw.initializeClaw(hardwareMap);
        DroneLauncher.initializeDroneLauncher(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean armDown = true;

        waitForStart();

        while (opModeIsActive()) {
            Localizer.update();

            Pose2d RobotPose = Localizer.getPoseEstimate();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double RobotHeading = RobotPose.getHeading();


            if (gamepad1.back) {
                RobotHeading = 0;
            }


            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-RobotHeading) - y * Math.sin(-RobotHeading);
            double rotY = x * Math.sin(-RobotHeading) + y * Math.cos(-RobotHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator);
            double backLeftPower = ((rotY - rotX + rx) / denominator);
            double frontRightPower = ((rotY - rotX - rx) / denominator);
            double backRightPower = ((rotY + rotX - rx) / denominator);

            /*if (gamepad1.dpad_up) {
                Driving_Speed = 0.85;
            }
            else if (gamepad1.dpad_down) {
                Driving_Speed = 0.5;
            }

             */


            frontLeftMotor.setPower(frontLeftPower * Driving_Speed);
            backLeftMotor.setPower(backLeftPower * Driving_Speed);
            frontRightMotor.setPower(frontRightPower * Driving_Speed);
            backRightMotor.setPower(backRightPower * Driving_Speed);
            

            telemetry.addData("Robot Heading: ", RobotHeading);

            //MecanumDrivebase.setDrivePower(new Pose2d(x, x, RobotHeading));

            if (gamepad1.y) {
                Claw.Actuate_Claw_Top_Finger("toggle");
            }
            if (gamepad1.a) {
                Claw.Actuate_Claw_Bottom_Finger("toggle");
            }

            if (gamepad1.left_bumper) {
                DroneLauncher.launchDrone();
            }

            if ((Arm.Arm_Motor.getCurrentPosition() > 530)) {
                Arm.setArmPosTo(525, 0.1);
            }
            if (Arm.Arm_Motor.getCurrentPosition() < 5) {
                Arm.setArmPosTo(5, 0.1);
            }

            //Smart TeleOp

            /*if (gamepad1.a){
                armSpeed = 0.2;
            }
            else if(gamepad1.b){
                armSpeed = 0.15;
            }
             */

            if (gamepad1.x) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Arm.setArmPosTo(525, armSpeed);
            }

            if (gamepad1.left_trigger > 0) {
                Claw.Actuate_Claw_Bottom_Finger("open");
                sleep(850);

                Driving_Speed = 0.25;
                backLeftMotor.setPower(Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(Driving_Speed);
                sleep(800);

                Driving_Speed = 0.85;
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);

                Claw.Actuate_Claw_Top_Finger("open");
            }
// genshin uid: 642041765
// add me pls !!
            if (gamepad1.b) {
                armDown = false;
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Arm.setArmPosTo(100, 0.15);
            }

            if (gamepad1.right_trigger > 0) {
                Driving_Speed = 0.1;
                armDown = true;
                Claw.Actuate_Claw_Bottom_Finger("open");
                Claw.Actuate_Claw_Top_Finger("open");
                sleep(700);
                Arm.setArmPosTo(5, 0.15);
                while (Arm.Arm_Motor.isBusy()) {}
                backLeftMotor.setPower(-Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(-Driving_Speed);

                sleep(350);

                Arm.setArmPosTo(5, 0.15);

                while (!Arm.Arm_Motor.isBusy()) {
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    break;
                }

                Driving_Speed = 0.85;
            }

            if (gamepad1.dpad_up) {
                Driving_Speed = 0.1;
                backLeftMotor.setPower(-Driving_Speed);
                backRightMotor.setPower(-Driving_Speed);
                frontLeftMotor.setPower(-Driving_Speed);
                frontRightMotor.setPower(-Driving_Speed);

                sleep(1000);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);

                Driving_Speed = 0.85;
            }

            if (!Arm.Arm_Motor.isBusy() && armDown) {
                armDown = false;
                sleep(50);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(150);
                Arm.setArmPosTo(100, 0.15);
            }

            Arm.setArmPosTo(Arm.getCurrentArmPos(), armSpeed);

            telemetry.addData("Arm is busy: ", Arm.Arm_Motor.isBusy());
            telemetry.addData("Arm Position: ", Arm.Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Target Position: ", Arm.Arm_Motor.getTargetPosition());
            telemetry.addData("Top Claw Position: ", Claw.Claw_Top_Finger.getPosition());
            telemetry.addData("Bottom Claw Position: ", Claw.Claw_Bottom_Finger.getPosition());
            telemetry.addData("Drone Launcher Position: ", DroneLauncher.DroneLauncherServo.getPosition());
            telemetry.addLine("");
            telemetry.addLine("Genshin UID: 642041765");
            telemetry.update();
        }
    }
}
