package org.firstinspires.ftc.teamcode.mechanisms.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class _2024011303_Kavi_Gupta_Arm_PID_V3 extends LinearOpMode {
    double armKp = 0.0398;//0.07791;//0.029399999; 0.13
    double armKi = 0.001;
    double armKd = 0.001;

    double armKf = 0;

    double armTicksPerRevolution = 537.7;

    double ticksPerDegree = armTicksPerRevolution/360;

    double integralSum = 0;
    double lastError = 0;

    final int tolerance = 10;

    ElapsedTime timer = new ElapsedTime();

    int Arm_Up_Position = 555;

    final int Arm_Idle_Position = 100;

    int Arm_Target_Position = Arm_Up_Position;

    double adjustmentValue = 0.0001;

    double out = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();


    public void runOpMode() {
        waitForStart();
        DcMotor Arm_Motor = hardwareMap.get(DcMotor.class, "Arm_Motor");

        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm_Motor.setDirection(DcMotor.Direction.REVERSE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());


        while (opModeIsActive()) {
            if (isStopRequested()) {return;}
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
            if (gamepad1.back) {
                Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm_Motor.setTargetPosition(75);
                Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Arm_Motor.isBusy()) {};
                break;
            }
            if (gamepad1.start) {
                Arm_Target_Position = 555;
            }
            int Arm_Position = Arm_Motor.getCurrentPosition();
            double error = Arm_Target_Position - Arm_Position;

            telemetry.addLine("Controls:");
            telemetry.addLine("Kp: increase dpad_up decrease dpad_down");
            telemetry.addLine("Kd: increase dpad_left decrease dpad_right");
            telemetry.addLine("Ki: increase a decrease x");
            telemetry.addData("Kp: ", armKp);
            telemetry.addData("Ki: ", armKi);
            telemetry.addData("Kd: ", armKd);
            telemetry.addData("Position: ", Arm_Position);
            telemetry.addData("Target: ", Arm_Target_Position);
            telemetry.addData("Error: ", error);

            if (Math.abs(error) > tolerance) {
                double derivative = (error - lastError) /timer.seconds();

                integralSum = integralSum + (error * timer.seconds());

                out = (armKp * error) + (armKi * integralSum) + (armKd * derivative);

                Arm_Motor.setPower(out);

                lastError = error;

                telemetry.addData("Power: ", out);
                telemetry.addLine("PID Running");

                timer.reset();
            } else {
                if (Arm_Target_Position == Arm_Up_Position) {
                    Arm_Target_Position = Arm_Idle_Position;
                }
                else if (Arm_Target_Position == Arm_Idle_Position) {
                    Arm_Target_Position = Arm_Up_Position;
                }

            }
            telemetry.update();
        }
    }
}
