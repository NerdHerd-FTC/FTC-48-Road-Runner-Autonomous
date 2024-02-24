package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.DroneLauncherInstance;

@TeleOp
@Disabled
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
