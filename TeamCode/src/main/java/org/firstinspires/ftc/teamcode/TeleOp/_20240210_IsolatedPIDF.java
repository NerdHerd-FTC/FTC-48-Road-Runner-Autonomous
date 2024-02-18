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
public class _20240210_IsolatedPIDF extends LinearOpMode {
    @Override
    public void runOpMode(){
        Arm_Instance_With_PIDF Arm = new Arm_Instance_With_PIDF();

        Arm.Initialize_Arm_Instance(hardwareMap);

        waitForStart();

        Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;

        while (opModeIsActive()) {
            if (gamepad1.b) {
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Backboard;
            }
            if (gamepad1.x) {
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;
            }

            Arm.Update_Arm_Position_With_PIDF();

            telemetry.addData("Arm Position: ", Arm.Arm_Current_Position);
            telemetry.addData("Arm Target Position: ", Arm.Arm_Target_Angle);
            telemetry.update();
        }
    }
}

