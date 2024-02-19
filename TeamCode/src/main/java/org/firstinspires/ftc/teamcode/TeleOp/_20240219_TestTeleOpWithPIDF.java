package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.arm.Arm_Instance_With_PIDF;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.DroneLauncherInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class _20240219_TestTeleOpWithPIDF extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer Localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        Arm_Instance_With_PIDF Arm = new Arm_Instance_With_PIDF();
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
            if (gamepad1.b) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Backboard;
            }

            if (gamepad1.x) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                Is_Arm_Down = false;
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;
            }

            boolean isRobotMovingToPickupPixel = false;

            if (isRobotMovingToPickupPixel) {
                if (System.currentTimeMillis() >= PixelPickupMoveForwardStartTime + Move_Robot_To_Pickup_Pixel.duration()) {
                    isRobotMovingToPickupPixel = false;
                }

            }


            if (Is_Arm_Down) {
                Is_Arm_Down = false;
                sleep(50);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(150);
                Arm.Arm_Target_Angle = 100;
            }

            Arm.Update_Arm_Position_With_PIDF();

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

