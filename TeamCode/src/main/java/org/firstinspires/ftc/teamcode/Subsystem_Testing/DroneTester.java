package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class DroneTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DroneLauncherInstance DroneLauncher = new DroneLauncherInstance();
        DroneLauncher.initializeDroneLauncher(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                DroneLauncher.launchDrone();
            }
            telemetry.addData("Drone Launcher Position: ", DroneLauncher.DroneLauncherServo.getPosition());
            telemetry.update();
        }
    }
}
