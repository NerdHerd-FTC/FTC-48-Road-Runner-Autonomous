package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Front;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;

public class _2023112701_Kavi_Gupta_Red_Alliance_Front_V1  extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        Pose2d StartingCoordinates = new Pose2d(12, -60, 270);

        MecanumDrivebase.setPoseEstimate(StartingCoordinates);

        Trajectory MoveToPropScanningLocation = MecanumDrivebase.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        MecanumDrivebase.followTrajectory(MoveToPropScanningLocation);

        //Scan for props

    }
}
