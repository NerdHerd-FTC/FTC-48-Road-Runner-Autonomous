package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled

public class ClawServoTestersV2 extends LinearOpMode {
    public Servo Claw_Top_Finger;

    public Servo Claw_Bottom_Finger;

    public void initializeClaw(HardwareMap hardwareMap) {
        Claw_Top_Finger = hardwareMap.get(Servo.class, "Claw_Top_Finger");
        Claw_Bottom_Finger = hardwareMap.get(Servo.class, "Claw_Bottom_Finger");

        Claw_Top_Finger.setDirection(Servo.Direction.REVERSE);
        Claw_Bottom_Finger.setDirection(Servo.Direction.FORWARD);

        Claw_Top_Finger.scaleRange(0, 1);
        Claw_Bottom_Finger.scaleRange(0, 1);


    }

    public void runOpMode() {
        initializeClaw(hardwareMap);

        waitForStart();

        double Top_Claw_Target = 0;
        double Bottom_Claw_Target = 0;



        while (opModeIsActive()) {
            if (isStopRequested()) return;

            telemetry.addData("Top Claw Target: ", Top_Claw_Target);
            telemetry.addData("Top Claw Position: ", Claw_Top_Finger.getPosition());
            telemetry.addData("Bottom Claw Target", Bottom_Claw_Target);
            telemetry.addData("Bottom Claw Position", Claw_Bottom_Finger.getPosition());
            telemetry.update();

            if (gamepad1.y) {
                Top_Claw_Target = 0.3;
                Bottom_Claw_Target = 0.3;
                Claw_Top_Finger.setPosition(Top_Claw_Target);
                Claw_Bottom_Finger.setPosition(Bottom_Claw_Target);
            }
            if (gamepad1.a) {
                Top_Claw_Target = 0;
                Bottom_Claw_Target = 0;
                Claw_Top_Finger.setPosition(Top_Claw_Target);
                Claw_Bottom_Finger.setPosition(Bottom_Claw_Target);
            }

            if (Top_Claw_Target > 1) {
                Top_Claw_Target = 1;
            }
            if (Bottom_Claw_Target > 1) {
                Bottom_Claw_Target = 1;
            }
            if (Top_Claw_Target < 0) {
                Top_Claw_Target = 0;
            }
            if (Bottom_Claw_Target < 0) {
                Bottom_Claw_Target = 0;
            }




        }






    }
}
