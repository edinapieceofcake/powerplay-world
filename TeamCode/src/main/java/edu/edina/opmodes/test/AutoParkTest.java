package edu.edina.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(group = "drive")
@Disabled
public class AutoParkTest extends LinearOpMode {
    public static double DISTANCE = 41; // in
    public static double ANGLE = -10;
    public static double STRAFEDISTANCE = 20;
    public static int SLEEPTIME = 23;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(36, 56, Math.toRadians(90)));

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(36, 56, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(36,56 - DISTANCE, Math.toRadians(ANGLE)))
                .build();

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(trajectory.end())
                .lineToSplineHeading(new Pose2d(36,32, Math.toRadians(90)))
                .strafeTo(new Vector2d(36 + STRAFEDISTANCE,32))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        sleep(SLEEPTIME * 1000);

        drive.followTrajectorySequence(trajectory1);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
