package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Front;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class _2023112701_Kavi_Gupta_Red_Alliance_Front_V1  extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        TensorFlowInstance TensorFlow = new TensorFlowInstance();

        Pose2d StartingCoordinates = new Pose2d(12, -60, Math.toRadians(270));

        MecanumDrivebase.setPoseEstimate(StartingCoordinates);

        TensorFlow.IntitializeTensorFlow(hardwareMap);

        String PropLocation;
        /*

        Trajectory MoveToPropScanningLocation = MecanumDrivebase.trajectoryBuilder(StartingCoordinates, true)
                .forward(12)
                .build();

        Trajectory MoveToCenterSpike = MecanumDrivebase.trajectoryBuilder(MoveToPropScanningLocation.end(), true)
                .strafeRight(5)
                .forward(12)
                .build();

        Trajectory MoveToBackBoardFromCenterSpike = MecanumDrivebase.trajectoryBuilder(MoveToCenterSpike.end(), true)
                .back(5)
                .strafeRight(30)
                .build();

        Trajectory MoveToLeftSpike = MecanumDrivebase.trajectoryBuilder(MoveToPropScanningLocation.end(), true)
                        .build();

         */

        TrajectorySequence MoveToPropScanningLocation = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .setReversed(true)
                .forward(12)
                .build();

        TensorFlow.SetWebcamStreamStatus("start");

        TrajectorySequence PlacePixelOnCenterSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .setReversed(true)
                .strafeRight(5)
                .forward(12)
                .addDisplacementMarker(() -> {
                    //claw release
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence PlaceYellowPixelOnCenterBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnCenterSpike.end())
                .setReversed(true)
                .back(5)
                .strafeRight(30)
                .turn(Math.toRadians(-270))
                .addDisplacementMarker(() -> {
                    //release pixel
                })
                .build();


        waitForStart();

        if (isStopRequested()) return;



        MecanumDrivebase.followTrajectorySequence(MoveToPropScanningLocation);

        PropLocation = TensorFlow.DetectProps();

        if (PropLocation == "left") {
            //do
        } else if (PropLocation == "center") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnCenterSpike);
            sleep(1500);
            MecanumDrivebase.followTrajectorySequenceAsync(PlaceYellowPixelOnCenterBackDrop);

        } else if (PropLocation == "right") {
            //do
        } else {
            //failsafe
        }




        //Scan for props

    }
}
