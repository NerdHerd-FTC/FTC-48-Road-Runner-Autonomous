package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Servo Test", group = "2023-12-01")
public class ServoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawTop = hardwareMap.servo.get("Drone_Launcher_Servo");
        clawTop.setDirection(Servo.Direction.FORWARD);
        clawTop.scaleRange(0, 1);


        clawTop.setPosition(0.3);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                clawTop.setPosition(0);
                sleep(500);
                clawTop.setPosition(0.5);
                sleep(500);
                clawTop.setPosition(1);
            }

            telemetry.addData("Claw Top Position: ", clawTop.getPosition());
            telemetry.update();
        }
    }
}