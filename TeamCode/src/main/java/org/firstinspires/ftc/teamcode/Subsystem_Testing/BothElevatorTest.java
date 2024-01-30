package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BothElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        double directionMultiplier = 1;

        double ElevatorMotorCount = 0;


        DcMotor LeftElevatorMotor = hardwareMap.get(DcMotor.class, "Left_Elevator_Motor");
        CRServo LeftElevatorServo = hardwareMap.get(CRServo.class, "Left_Elevator_Servo");
        DcMotor RightElevatorMotor = hardwareMap.get(DcMotor.class, "Right_Elevator_Motor");
        CRServo RightElevatorServo = hardwareMap.get(CRServo.class, "Right_Elevator_Servo");
        LeftElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        RightElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            LeftElevatorMotor.setPower(0);
            LeftElevatorServo.setPower(0);
            RightElevatorMotor.setPower(0);
            RightElevatorServo.setPower(0);
            if (gamepad1.dpad_up) {
                directionMultiplier = 1;
                telemetry.addData("Direction Multiplier: ", directionMultiplier);
            }
            if (gamepad1.dpad_down) {
                directionMultiplier = -1;
                telemetry.addData("Direction Multiplier: ", directionMultiplier);
            }
            if (gamepad1.right_trigger > 0.1) {
                double triggerValue = gamepad1.right_trigger;
                LeftElevatorMotor.setPower(triggerValue*directionMultiplier);
                RightElevatorMotor.setPower(triggerValue*directionMultiplier);
                ElevatorMotorCount += triggerValue*directionMultiplier;
                telemetry.addLine("Motor Spinning");
            }
            if (gamepad1.left_trigger > 0.1) {
                LeftElevatorServo.setPower(gamepad1.left_trigger*directionMultiplier);
                RightElevatorServo.setPower(gamepad1.left_trigger*directionMultiplier);
                telemetry.addLine("Servo Working");
            }
            if (gamepad1.back) {
                LeftElevatorMotor.setPower(0);
                RightElevatorMotor.setPower(0);
                telemetry.addLine("Reset");
            }
            telemetry.update();
        }
    }
}

