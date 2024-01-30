package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Front;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance_With_PTZ;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled

public class _2023120805_Kavi_Gupta_Red_Alliance_Front_V5_No_Arm extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        TensorFlowInstance_With_PTZ TensorFlow = new TensorFlowInstance_With_PTZ();
        final TensorFlowInstance_With_PTZ.CameraStreamProcessor processor = new TensorFlowInstance_With_PTZ.CameraStreamProcessor();

        Pose2d StartingCoordinates = new Pose2d(12, -60, Math.toRadians(270));
        MecanumDrivebase.setPoseEstimate(StartingCoordinates);
        TensorFlow.InitializeTensorFlow(hardwareMap, processor);

        String PropLocation;
        TensorFlow.SetWebcamStreamStatus("start");


        TrajectorySequence MoveToPropScanningLocation = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .back(12)

                .waitSeconds(1)
                .build();

        TrajectorySequence PlacePixelOnCenterSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .strafeLeft(5)
                .back(20)

                .waitSeconds(1)

                .build();



        TrajectorySequence PlacePixelOnLeftSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .back(20)
                .turn(Math.toRadians(90))
                .back(2)

                .build();

        TrajectorySequence PlacePixelOnRightSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .back(22)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .back(10)

                .build();


        TrajectorySequence PlaceYellowPixelOnCenterBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnCenterSpike.end())
                .forward(5)
                .strafeLeft(20)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .forward(5)
                .waitSeconds(2)
                .back(5)
                .strafeRight(20)
                .forward(12)
                .build();

        TrajectorySequence PlaceYellowPixelOnLeftBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnLeftSpike.end())
                .forward(28)
                .waitSeconds(3)
                .forward(5)
                .waitSeconds(1)
                .back(5)
                .strafeRight(30)
                .forward(15)
                .build();

        TrajectorySequence PlaceYellowPixelOnRightBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnRightSpike.end())
                .forward(10)
                .turn(Math.toRadians(180))
                .waitSeconds(1)
                .strafeRight(16)
                .forward(27)
                .waitSeconds(3)
                .back(5)
                .waitSeconds(1)
                .back(10)
                .strafeRight(17)
                .forward(20)
                .build();

        TrajectorySequence FailsafePark = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .strafeLeft(40)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        MecanumDrivebase.followTrajectorySequence(MoveToPropScanningLocation);

        TensorFlow.SetWebcamStreamStatus("start");

        PropLocation = "right";

        if (PropLocation == "left") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnLeftSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnLeftBackDrop);
        } else if (PropLocation == "center") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnCenterSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnCenterBackDrop);

        } else if (PropLocation == "right") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnRightSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnRightBackDrop);
        } else {
            MecanumDrivebase.followTrajectorySequence(FailsafePark);
        }




        //Scan for props

    }
}
