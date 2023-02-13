package edu.edina.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import edu.edina.library.util.PoseStorage;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class DriveToPole extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)));
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                .forward(25)
                .splineTo(new Vector2d(-30,-10), Math.toRadians(-180))
                .build();

        drive.followTrajectorySequence(traj);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
