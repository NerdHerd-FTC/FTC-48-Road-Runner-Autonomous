package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;

@Autonomous
@Disabled


public class ArmMotorTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        ArmInstance Arm = new ArmInstance();

        Arm.initializeArm(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Arm.setArmMotorPower(1                                                                                                                           );

        Arm.moveArmTo(400);

        while (Arm.Arm_Motor.isBusy()) {};

        Arm.setArmMotorPower(1);

        Arm.moveArmTo(100);

        while (Arm.Arm_Motor.isBusy()) {}

    }
}

