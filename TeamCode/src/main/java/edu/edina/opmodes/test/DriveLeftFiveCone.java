package edu.edina.opmodes.test;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
public class DriveLeftFiveCone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double waitTime = 1.5;

        drive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(270)));
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                .splineTo(new Vector2d(28, 10), Math.toRadians(245))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(58, 18, Math.toRadians(180)))
                .build();

        TrajectorySequence strafe = drive.trajectorySequenceBuilder(traj.end())
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 10;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 10;
                    }
                })
                .strafeLeft(5).build();

        TrajectorySequence forward = drive.trajectorySequenceBuilder(new Pose2d(56, 12, Math.toRadians(180)))
                .resetConstraints()
                .lineToSplineHeading(new Pose2d(28, 10, Math.toRadians(245)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(58, 20, Math.toRadians(180)))
                .build();

        waitForStart();

        if (isStopRequested()) return;
/*
        drive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(270)));
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                .splineTo(new Vector2d(28, 10), Math.toRadians(245))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(58, 18, Math.toRadians(180)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(28, 10, Math.toRadians(245)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(58, 20, Math.toRadians(180)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(28, 10, Math.toRadians(245)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(58, 22, Math.toRadians(180)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(28, 10, Math.toRadians(245)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(58, 24, Math.toRadians(180)))
                .waitSeconds(waitTime)
                .lineToSplineHeading(new Pose2d(28, 10, Math.toRadians(245)))
                .build();
*/
        //drive.followTrajectorySequence(traj);

        long startTime = System.currentTimeMillis();
        drive.followTrajectorySequenceAsync(strafe);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();

            if (System.currentTimeMillis() > (startTime + 500))
                break;

            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.update();
        }

        //drive.setPoseEstimate(new Pose2d(58, 12, Math.toRadians(180)));
        //drive.followTrajectorySequence(forward);

        PoseStorage.currentPose = drive.getPoseEstimate();
        telemetry.addData("Pose", drive.getPoseEstimate());
        telemetry.update();
    }
}
