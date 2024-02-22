package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class _20240219_TeleOpNoProcedures_V1 extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer Localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        Localizer.setPoseEstimate(MecanumDrivebase.getPoseEstimate());

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

        Servo Claw_Top_Finger = hardwareMap.get(Servo.class, "Claw_Top_Finger");
        Servo Claw_Bottom_Finger = hardwareMap.get(Servo.class, "Claw_Bottom_Finger");

        double Drone_Launcher_Idle_Position = 0.3;

        double Drone_Launcher_Launch_Position = 1;

        Servo DroneLauncherServo = hardwareMap.get(Servo.class, "Drone_Launcher_Servo");
        DroneLauncherServo.setDirection(Servo.Direction.FORWARD);
        DroneLauncherServo.scaleRange(0, 1);
        DroneLauncherServo.setPosition(Drone_Launcher_Idle_Position);

        int Arm_Target_Position_Ticks_For_Backboard = 535;
        int Arm_Target_Position_Ticks_For_Idle_Position = 100;
        double Robot_Driving_Speed = 0.9;
        double Claw_Bottom_Open_Position_Value = 0.1;

        double Claw_Bottom_Close_Position_Value = 0.2;

        double Claw_Top_Open_Position_Value = 0.2;

        double Claw_Top_Close_Position_Value = 0.4;

        double P_Coefficient = 0.0025;
        double I_Coefficient = 0.12;
        double D_Coefficient = 0.001;
        double F_Coefficient = 0.12;

        String Claw_Top_Status = "";
        String Claw_Bottom_Status = "";

        int Arm_Target_Angle = 0;
        int Arm_Positional_Error;
        int Arm_Motor_Tolerance = 0; //ticks
        int Arm_Current_Position;

        double CalculatedPID;
        double CalculatedFeedForward;
        double Calculated_Power_For_Arm_Motor =0;

        double Arm_Ticks_Per_Revolution = 537.7; //goBilda Yellow Jacket 5203
        double Arm_Motor_Ticks_Per_Degree = Arm_Ticks_Per_Revolution / 180;

        DcMotorEx Arm_Motor;

        PIDController Arm_PID_Controller = new PIDController(P_Coefficient, I_Coefficient, D_Coefficient);
        Arm_Motor = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        Arm_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        Arm_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Claw_Top_Finger.setDirection(Servo.Direction.REVERSE);
        Claw_Bottom_Finger.setDirection(Servo.Direction.FORWARD);

        Claw_Top_Finger.scaleRange(0, 1);
        Claw_Bottom_Finger.scaleRange(0, 1);

        boolean Is_Arm_Down = true;
        int Arm_Down_Count = 0;

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

        Arm_Target_Angle = 100;
        boolean isRobotMovingToPickupPixel = false;

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

            if (gamepad1.right_bumper) {
                DroneLauncherServo.setPosition(Drone_Launcher_Launch_Position);
                sleep(1500);
                DroneLauncherServo.setPosition(Drone_Launcher_Idle_Position);
            }

            if (gamepad1.b) {
                Claw_Bottom_Finger.setPosition(Claw_Bottom_Close_Position_Value);
                Claw_Top_Finger.setPosition(Claw_Top_Close_Position_Value);
                Arm_Target_Angle = Arm_Target_Position_Ticks_For_Backboard;
            }

            if (gamepad1.right_trigger > 0) {
                Claw_Bottom_Finger.setPosition(Claw_Bottom_Open_Position_Value);

                Trajectory Move_Robot_To_Release_Next_Pixel = MecanumDrivebase.trajectoryBuilder(Current_Robot_Pose)
                        .strafeLeft(2)
                        .build();

                MecanumDrivebase.followTrajectoryAsync(Move_Robot_To_Release_Next_Pixel);

                Claw_Top_Finger.setPosition(Claw_Top_Open_Position_Value);
            }

            if (gamepad1.x) {
                Claw_Bottom_Finger.setPosition(Claw_Bottom_Close_Position_Value);
                Claw_Top_Finger.setPosition(Claw_Top_Close_Position_Value);
                Is_Arm_Down = false;
                Arm_Target_Angle = Arm_Target_Position_Ticks_For_Idle_Position;
            }


            if (gamepad1.left_trigger > 0) {
                Is_Arm_Down = true;
                Claw_Bottom_Finger.setPosition(Claw_Bottom_Open_Position_Value);
                Claw_Top_Finger.setPosition(Claw_Top_Open_Position_Value);
                sleep(700);

                Move_Robot_To_Pickup_Pixel = MecanumDrivebase.trajectorySequenceBuilder(Current_Robot_Pose)
                        .setVelConstraint(MecanumDrivebaseInstance.getVelocityConstraint(5, 10, DriveConstants.TRACK_WIDTH))
                        .forward(2)
                        .build();

                MecanumDrivebase.followTrajectorySequenceAsync(Move_Robot_To_Pickup_Pixel);
                PixelPickupMoveForwardStartTime = System.currentTimeMillis();
                isRobotMovingToPickupPixel = true;
                Arm_Target_Angle = 0;
            }

            if (isRobotMovingToPickupPixel) {
                if (System.currentTimeMillis() >= PixelPickupMoveForwardStartTime + Move_Robot_To_Pickup_Pixel.duration()) {
                    isRobotMovingToPickupPixel = false;
                }

            }

            if (Is_Arm_Down) {
                Arm_Down_Count += 1;
                if (Arm_Down_Count > 150) {
                    Is_Arm_Down = false;
                    Arm_Down_Count = 0;
                    //sleep(50);
                    Claw_Bottom_Finger.setPosition(Claw_Bottom_Close_Position_Value);
                    Claw_Top_Finger.setPosition(Claw_Top_Close_Position_Value);
                    //sleep(150);
                    Arm_Target_Angle = Arm_Target_Position_Ticks_For_Idle_Position;
                }
            }

            if (gamepad2.dpad_up) {
                directionMultiplier = 1;
            }
            if (gamepad2.dpad_down) {
                directionMultiplier = -1;
            }
            if (gamepad2.right_trigger > 0.1) {
                double triggerValue = gamepad2.right_trigger;
                LeftElevatorMotor.setPower(triggerValue * directionMultiplier);
                RightElevatorMotor.setPower(triggerValue * directionMultiplier);
            }
            if (gamepad2.left_trigger > 0.1) {
                LeftElevatorServo.setPower(gamepad2.left_trigger * directionMultiplier);
                RightElevatorServo.setPower(gamepad2.left_trigger * directionMultiplier);
            }

            //Arm_PID_Controller.setPID(P_Coefficient, I_Coefficient, D_Coefficient);
            Arm_Current_Position = Arm_Motor.getCurrentPosition();
            //Arm_Positional_Error = Arm_Target_Angle - Arm_Current_Position;
            CalculatedPID = Arm_PID_Controller.calculate(Arm_Current_Position, Arm_Target_Angle);
            CalculatedFeedForward = Math.cos(Math.toRadians(Arm_Target_Angle / Arm_Motor_Ticks_Per_Degree)) * F_Coefficient;
            Calculated_Power_For_Arm_Motor = CalculatedPID + CalculatedFeedForward;
            Arm_Motor.setPower(Calculated_Power_For_Arm_Motor);
            /*if (Math.abs(Arm_Positional_Error) > Arm_Motor_Tolerance) {
                CalculatedPID = Arm_PID_Controller.calculate(Arm_Current_Position, Arm_Target_Angle);
                CalculatedFeedForward = Math.cos(Math.toRadians(Arm_Target_Angle / Arm_Motor_Ticks_Per_Degree)) * F_Coefficient;
                Calculated_Power_For_Arm_Motor = CalculatedPID + CalculatedFeedForward;
                Arm_Motor.setPower(Calculated_Power_For_Arm_Motor);
            }
            Arm_Motor.setPower(Calculated_Power_For_Arm_Motor);

             */


            if (gamepad1.dpad_up) {
                frontLeftPower = -0.1 / Robot_Driving_Speed;
                backLeftPower = -0.1 / Robot_Driving_Speed;
                frontRightPower = -0.1 / Robot_Driving_Speed;
                backRightPower = -0.1 / Robot_Driving_Speed;
            }

            frontLeftMotor.setPower(frontLeftPower * Robot_Driving_Speed);
            backLeftMotor.setPower(backLeftPower * Robot_Driving_Speed);
            frontRightMotor.setPower(frontRightPower * Robot_Driving_Speed);
            backRightMotor.setPower(backRightPower * Robot_Driving_Speed);

            telemetry.addData("Arm Position: ", Arm_Current_Position);
            telemetry.addData("Arm Target Position: ", Arm_Target_Angle);
            telemetry.addData("Top Claw Position: ", Claw_Top_Finger.getPosition());
            telemetry.addData("Bottom Claw Position: ", Claw_Bottom_Finger.getPosition());
            telemetry.addData("Drone Launcher Position: ", DroneLauncherServo.getPosition());
            telemetry.addData("Direction Multiplier: ", directionMultiplier);
            telemetry.addLine("--- Robot Pos ---");
            telemetry.addData("x: ", currentX);
            telemetry.addData("y: ", currentY);
            telemetry.update();
        }
    }
}


