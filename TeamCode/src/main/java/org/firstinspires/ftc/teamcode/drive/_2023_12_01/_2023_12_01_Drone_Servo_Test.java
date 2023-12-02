package org.firstinspires.ftc.teamcode.drive._2023_12_01;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Servo Test", group = "2023-12-01")
public class _2023_12_01_Drone_Servo_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawTop = hardwareMap.servo.get("Drone_Launcher_Servo");
        clawTop.setDirection(Servo.Direction.FORWARD);
        clawTop.scaleRange(0, 1);

        waitForStart();
        clawTop.setPosition(0.3);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                clawTop.setPosition(0);
                wait(500);
                clawTop.setPosition(0.5);
                wait(500);
                clawTop.setPosition(1);
            }

            telemetry.addData("Claw Top Position: ", clawTop.getPosition());
            telemetry.update();
        }
    }
}