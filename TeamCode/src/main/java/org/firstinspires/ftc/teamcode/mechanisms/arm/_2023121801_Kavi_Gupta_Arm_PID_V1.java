package org.firstinspires.ftc.teamcode.mechanisms.arm;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class _2023121801_Kavi_Gupta_Arm_PID_V1 extends LinearOpMode {
    double armKp = 0.13;//0.029399999; 0.13
    double armKi = 0.00005999999;
    double armKd = 0.00001;

    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    int Arm_Target_Position = 555;

    double adjustmentValue = 0.000001;





    public void runOpMode() {
        waitForStart();
        DcMotor Arm_Motor = hardwareMap.get(DcMotor.class, "Arm_Motor");

        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm_Motor.setDirection(DcMotor.Direction.REVERSE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeIsActive()) {
            if (isStopRequested()) {return;}
            while (Arm_Motor.getCurrentPosition() != Arm_Target_Position) {
                if (isStopRequested()) { break;}
                if (gamepad1.dpad_up) {
                    armKp += 0.0001;
                }
                if (gamepad1.dpad_down) {
                    armKp -= 0.0001;
                    if (armKp < 0) {
                        armKp = 0;
                    }
                }
                if (gamepad1.dpad_right) {
                    armKd += adjustmentValue;
                }
                if (gamepad1.dpad_left) {
                    armKd -= adjustmentValue;
                    if (armKd < 0) {
                        armKd = 0;
                    }
                }
                if (gamepad1.y) {
                    armKi += adjustmentValue;
                }
                if (gamepad1.a) {
                    armKi -= adjustmentValue;
                    if (armKi < 0) {
                        armKi = 0;
                    }
                }
                telemetry.addLine("Controls:");
                telemetry.addLine("Kp: increase dpad_up decrease dpad_down");
                telemetry.addLine("Kd: increase dpad_left decrease dpad_right");
                telemetry.addLine("Ki: increase a decrease x");

                double Arm_Position = Arm_Motor.getCurrentPosition();

                double error = Arm_Target_Position - Arm_Position;

                double derivative = (error - lastError) /timer.seconds();

                integralSum = integralSum + (error * timer.seconds());

                double out = (armKp * error) + (armKi * integralSum) + (armKd * derivative);

                Arm_Motor.setPower(out);

                lastError = error;

                telemetry.addData("Kp: ", armKp);
                telemetry.addData("Ki: ", armKi);
                telemetry.addData("Kd: ", armKd);
                telemetry.addData("Position: ", Arm_Position);
                telemetry.addData("Target: ", Arm_Target_Position);
                telemetry.addData("Power: ", out);
                telemetry.update();

                timer.reset();
            }


        }
    }
}
