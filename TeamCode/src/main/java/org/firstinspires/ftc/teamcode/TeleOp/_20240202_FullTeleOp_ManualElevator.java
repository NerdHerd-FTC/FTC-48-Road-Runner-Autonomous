package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.arm.Arm_Instance_With_PIDF_And_Power_To_0;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.DroneLauncherInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Disabled
public class _20240202_FullTeleOp_ManualElevator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer Localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        Arm_Instance_With_PIDF_And_Power_To_0 Arm = new Arm_Instance_With_PIDF_And_Power_To_0();
        ClawInstance Claw = new ClawInstance();
        DroneLauncherInstance DroneLauncher = new DroneLauncherInstance();

        Localizer.setPoseEstimate(MecanumDrivebase.getPoseEstimate());

        Arm.Initialize_Arm_Instance(hardwareMap);
        Claw.initializeClaw(hardwareMap);
        DroneLauncher.initializeDroneLauncher(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor LeftElevatorMotor = hardwareMap.get(DcMotor.class, "Left_Elevator_Motor");
        CRServo LeftElevatorServo = hardwareMap.get(CRServo.class, "Left_Elevator_Servo");
        DcMotor RightElevatorMotor = hardwareMap.get(DcMotor.class, "Right_Elevator_Motor");
        CRServo RightElevatorServo = hardwareMap.get(CRServo.class, "Right_Elevator_Servo");

        LeftElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        RightElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean Is_Arm_Down = true;

        boolean Is_Robot_In_Slo_Mo = false;

        boolean Is_Power_Set_To_Zero = false;

        boolean Lower_Arm_For_Solo_Pixels = false;

        double directionMultiplier = 1;


        long PixelPickupMoveForwardStartTime = 0;
        long Robot_Slo_Mo_Start_Time = 0;

        TrajectorySequence Move_Robot_Forward_For_Slo_Mo = null;
        TrajectorySequence Move_Robot_To_Pickup_Pixel = null;

        double currentX = 0;
        double currentY = 0;

        waitForStart();

        Claw.Actuate_Claw_Top_Finger("close");
        Claw.Actuate_Claw_Bottom_Finger("close");

        Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;

        while (opModeIsActive()) {
            Localizer.update();

            Pose2d Current_Robot_Pose = Localizer.getPoseEstimate();

            double Left_Stick_Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double Left_Stick_X = gamepad1.left_stick_x;
            double Right_Stick_X = gamepad1.right_stick_x;

            double Current_Robot_Heading = Current_Robot_Pose.getHeading();
            currentX = Current_Robot_Pose.getX();
            currentY = Current_Robot_Pose.getY();


            if (gamepad1.back) {
                Localizer.setPoseEstimate(new Pose2d(Current_Robot_Pose.getX(), Current_Robot_Pose.getY(), 0)); //Reset Heading
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = Left_Stick_X * Math.cos(-Current_Robot_Heading) - Left_Stick_Y * Math.sin(-Current_Robot_Heading);
            double rotY = Left_Stick_X * Math.sin(-Current_Robot_Heading) + Left_Stick_Y * Math.cos(-Current_Robot_Heading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(Right_Stick_X), 1);
            double frontLeftPower = ((rotY + rotX + Right_Stick_X) / denominator);
            double backLeftPower = ((rotY - rotX + Right_Stick_X) / denominator);
            double frontRightPower = ((rotY - rotX - Right_Stick_X) / denominator);
            double backRightPower = ((rotY + rotX - Right_Stick_X) / denominator);

            frontLeftMotor.setPower(frontLeftPower * TeleOpConstants.Robot_Driving_Speed);
            backLeftMotor.setPower(backLeftPower * TeleOpConstants.Robot_Driving_Speed);
            frontRightMotor.setPower(frontRightPower * TeleOpConstants.Robot_Driving_Speed);
            backRightMotor.setPower(backRightPower * TeleOpConstants.Robot_Driving_Speed);

            if (gamepad1.y) {
                Claw.Actuate_Claw_Top_Finger("toggle");
            }
            if (gamepad1.a) {
                Claw.Actuate_Claw_Bottom_Finger("toggle");
            }

            if (gamepad1.right_bumper) {
                DroneLauncher.launchDrone();
            }

            if (gamepad1.b) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Backboard;
            }

            if (gamepad1.right_trigger > 0) {
                Claw.Actuate_Claw_Bottom_Finger("open");

                Trajectory Move_Robot_To_Release_Next_Pixel = MecanumDrivebase.trajectoryBuilder(Current_Robot_Pose)
                        .strafeLeft(2)
                        .build();

                MecanumDrivebase.followTrajectoryAsync(Move_Robot_To_Release_Next_Pixel);

                Claw.Actuate_Claw_Top_Finger("open");
            }

            if (gamepad1.x) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Is_Arm_Down = false;
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;
            }

            boolean isRobotMovingToPickupPixel = false;
            if (gamepad1.left_trigger > 0) {
                Is_Arm_Down = true;
                Claw.Actuate_Claw_Bottom_Finger("open");
                Claw.Actuate_Claw_Top_Finger("open");
                sleep(700);

                Move_Robot_To_Pickup_Pixel = MecanumDrivebase.trajectorySequenceBuilder(Current_Robot_Pose)
                        .setVelConstraint(MecanumDrivebaseInstance.getVelocityConstraint(5, 10, DriveConstants.TRACK_WIDTH))
                        .forward(2)
                        .build();

                MecanumDrivebase.followTrajectorySequenceAsync(Move_Robot_To_Pickup_Pixel);
                PixelPickupMoveForwardStartTime = System.currentTimeMillis();
                isRobotMovingToPickupPixel = true;

                if (currentX < -24 && currentY > -36 && currentY < 36) {
                    Arm.SetPowerToZero();
                    Is_Power_Set_To_Zero = true;
                } else {
                    Arm.Arm_Target_Angle = 5;
                }
            }

            if (isRobotMovingToPickupPixel) {
                if (System.currentTimeMillis() >= PixelPickupMoveForwardStartTime + Move_Robot_To_Pickup_Pixel.duration()) {
                    isRobotMovingToPickupPixel = false;
                }

            }


            if (gamepad1.left_bumper) {
                Lower_Arm_For_Solo_Pixels = true;
                Arm.Arm_Target_Angle = 25;

                if (isRobotMovingToPickupPixel) {
                    if (System.currentTimeMillis() >= PixelPickupMoveForwardStartTime + Move_Robot_To_Pickup_Pixel.duration()) {
                        Is_Robot_In_Slo_Mo = false;
                    }
                }

                Move_Robot_To_Pickup_Pixel = MecanumDrivebase.trajectorySequenceBuilder(Current_Robot_Pose)
                        .setVelConstraint(MecanumDrivebaseInstance.getVelocityConstraint(5, 10, DriveConstants.TRACK_WIDTH))
                        .forward(2)
                        .build();

                MecanumDrivebase.followTrajectorySequenceAsync(Move_Robot_To_Pickup_Pixel);
                PixelPickupMoveForwardStartTime = System.currentTimeMillis();
                isRobotMovingToPickupPixel = true;
            }



            if (gamepad1.dpad_up) {

                if (Is_Robot_In_Slo_Mo == false) {
                    Move_Robot_Forward_For_Slo_Mo = MecanumDrivebase.trajectorySequenceBuilder(Current_Robot_Pose)
                            .setVelConstraint(MecanumDrivebaseInstance.getVelocityConstraint(5, 10, DriveConstants.TRACK_WIDTH))
                            .forward(2)
                            .build();
                    Robot_Slo_Mo_Start_Time = System.currentTimeMillis();
                    MecanumDrivebase.followTrajectorySequenceAsync(Move_Robot_Forward_For_Slo_Mo);
                    Is_Robot_In_Slo_Mo = true;
                }
            }

            if (Is_Robot_In_Slo_Mo) {
                if (System.currentTimeMillis() >= Robot_Slo_Mo_Start_Time + Move_Robot_Forward_For_Slo_Mo.duration()) {
                    Is_Robot_In_Slo_Mo = false;
                }
            }


            if (Lower_Arm_For_Solo_Pixels) {
                Lower_Arm_For_Solo_Pixels = false;
                Claw.Actuate_Claw_Bottom_Finger("open");
                Claw.Actuate_Claw_Top_Finger("open");
                Arm.Arm_Target_Angle = 5;
                Is_Arm_Down = true;
            }

            if (Is_Arm_Down) {
                Is_Arm_Down = false;
                Is_Power_Set_To_Zero = false;
                sleep(50);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(150);
                Arm.Arm_Target_Angle = 100;
            }

            if (gamepad2.dpad_up) {
                directionMultiplier = 1;
            }
            if (gamepad2.dpad_down) {
                directionMultiplier = -1;
            }
            if (gamepad2.right_trigger > 0.1) {
                double triggerValue = gamepad2.right_trigger;
                LeftElevatorMotor.setPower(triggerValue*directionMultiplier);
                RightElevatorMotor.setPower(triggerValue*directionMultiplier);
            }
            if (gamepad2.left_trigger > 0.1) {
                LeftElevatorServo.setPower(gamepad2.left_trigger*directionMultiplier);
                RightElevatorServo.setPower(gamepad2.left_trigger*directionMultiplier);
            }

            if (!Is_Power_Set_To_Zero) {
                Arm.Update_Arm_Position_With_PIDF();
            }

            telemetry.addData("Arm Position: ", Arm.Arm_Current_Position);
            telemetry.addData("Arm Target Position: ", Arm.Arm_Target_Angle);
            telemetry.addData("Top Claw Position: ", Claw.Claw_Top_Finger.getPosition());
            telemetry.addData("Bottom Claw Position: ", Claw.Claw_Bottom_Finger.getPosition());
            telemetry.addData("Drone Launcher Position: ", DroneLauncher.DroneLauncherServo.getPosition());
            telemetry.addData("Direction Multiplier: ", directionMultiplier);
            telemetry.addLine("--- Robot Pos ---");
            telemetry.addData("x: ", currentX);
            telemetry.addData("x: ", currentY);
            telemetry.update();
        }
    }
}

