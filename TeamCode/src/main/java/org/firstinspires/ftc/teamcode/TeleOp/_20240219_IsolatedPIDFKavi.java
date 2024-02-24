package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.arm.Arm_Instance_With_PIDF_Kavi;

@TeleOp
public class _20240219_IsolatedPIDFKavi extends LinearOpMode {
    @Override
    public void runOpMode(){
        Arm_Instance_With_PIDF_Kavi Arm = new Arm_Instance_With_PIDF_Kavi();

        Arm.Initialize_Arm_Instance(hardwareMap);

        waitForStart();

        Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;

        while (opModeIsActive()) {
            if (gamepad1.b) {
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Backboard;
            }
            if (gamepad1.x) {
                Arm.Arm_Target_Angle = TeleOpConstants.Arm_Target_Position_Ticks_For_Idle_Position;
            }

            Arm.Update_Arm_Position_With_PIDF();

            telemetry.addData("Arm Position: ", Arm.Arm_Current_Position);
            telemetry.addData("Arm Target Position: ", Arm.Arm_Target_Angle);
            telemetry.update();
        }
    }
}

