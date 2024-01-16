package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class DriveBaseTesting  extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motorFL =  hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR =  hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL =  hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR =  hardwareMap.get(DcMotor.class, "motorBR");

        String direction = "front";

        waitForStart();

        while (opModeIsActive()) {
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFR.setPower(0);
            motorFL.setPower(0);
            if (gamepad1.dpad_up) {
                direction = "front";
            }
            if (gamepad1.dpad_down) {
                direction = "back";
            }
            if (gamepad1.left_trigger > 0.1) {
                if (direction == "front") {
                    motorFL.setPower(gamepad1.left_trigger);
                    telemetry.addLine("Testing Motor FL");
                } else if (direction == "back") {
                    motorBL.setPower(gamepad1.left_trigger);
                    telemetry.addLine("Testing Motor BL");
                }
            }
            if (gamepad1.right_trigger > 0.1) {
                if (direction == "front") {
                    motorFR.setPower(gamepad1.right_trigger);
                    telemetry.addLine("Testing Motor FR");
                } else if (direction == "back") {
                    motorBR.setPower(gamepad1.right_trigger);
                    telemetry.addLine("Testing Motor BR");
                }
            }
            telemetry.update();
        }

    }
}
