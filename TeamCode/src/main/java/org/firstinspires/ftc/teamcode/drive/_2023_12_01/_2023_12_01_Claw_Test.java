package org.firstinspires.ftc.teamcode.drive._2023_12_01;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Claw Test", group = "2023-12-01")
public class _2023_12_01_Claw_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawTop = hardwareMap.servo.get("clawTop");
        Servo clawBottom = hardwareMap.servo.get("clawBottom");

        clawTop.setDirection(Servo.Direction.FORWARD);
        clawBottom.setDirection(Servo.Direction.REVERSE);

        clawTop.scaleRange(0, 1);
        clawBottom.scaleRange(0, 1);

        waitForStart();

        clawTop.setPosition(0.3);
        clawBottom.setPosition(0.2);

        while (opModeIsActive()) {
            if (gamepad1.y && clawTop.getPosition() != 0) {
                clawTop.setPosition(clawTop.getPosition() - 0.1);
            }
            if (gamepad1.x && clawTop.getPosition() != 0) {
                clawTop.setPosition(clawTop.getPosition() + 0.1);
            }

            if (gamepad1.b && clawBottom.getPosition() != 0) {
                clawBottom.setPosition(clawBottom.getPosition() - 0.1);
            }
            if (gamepad1.a && clawBottom.getPosition() != 0) {
                clawBottom.setPosition(clawBottom.getPosition() + 0.1);
            }

            telemetry.addData("Claw Top Position: ", clawTop.getPosition());
            telemetry.addData("Claw Bottom Position: ", clawBottom.getPosition());

            telemetry.update();
        }
    }
}