package org.firstinspires.ftc.teamcode.drive._2023_12_01;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drone Test")
public class _2023_12_01_Drone_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo droneServo = hardwareMap.servo.get("drone");
        droneServo.setDirection(Servo.Direction.FORWARD);
        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.7);


        while (opModeIsActive()) {
            double droneServoPosition = droneServo.getPosition();

            if (gamepad1.y) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.7);
            }
            telemetry.addData("Drone Servo Position: ", droneServoPosition);
            telemetry.update();
        }
    }
}