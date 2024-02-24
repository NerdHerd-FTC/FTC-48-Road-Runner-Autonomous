package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled



@Autonomous
public class Trajectory_Sequence_Tester extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        TrajectorySequence TestTrajectory = MecanumDrivebase.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(270)))
                .setReversed(true)
                .forward(5)
                .back(5)
                .strafeRight(5)
                .strafeLeft(5)
                .turn(Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        MecanumDrivebase.followTrajectorySequence(TestTrajectory);

    }
}
