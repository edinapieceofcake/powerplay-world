package edu.edina.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.library.util.ClawServoPosition;

@Autonomous(group = "Simple")@Config
public class SrafeTest extends AutoBase {

    protected String getCameraName() {
        return "highCamera";
    }

    @Override
    protected void addAdditionalTelemetry(Telemetry telemetry) {
        telemetry.addData("Make sure claw is in the front and high   camera is facing field.", "");
        telemetry.addData("Cone should always be on side with medium pole", "");
    }

    protected boolean shouldClawBeInTheFront() {
        return true;
    }

    protected Pose2d getStartPose() { return new Pose2d(-50, -65, Math.toRadians(0)); }

    @Override
    public void initPaths() {
        Vector2d startEndPoint = new Vector2d(-33, -30);
        Vector2d endEndPoint = new Vector2d(-50, -65);

        start = drive.trajectorySequenceBuilder(getStartPose())
                .strafeTo(startEndPoint)
                .build();

        backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeTo(endEndPoint)
                .build();

        backToDropOff1 = drive.trajectorySequenceBuilder(getStartPose())
                .strafeLeft(24)
                .build();

        backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeRight(24)
                .build();

        backToDropOff2 = drive.trajectorySequenceBuilder(getStartPose())
                .forward(24)
                .build();

        backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .back(24)
                .build();
    }

    @Override
    protected void runPaths() {

        drive.setPoseEstimate(getStartPose());

        drive.followTrajectorySequence(backToDropOff1);
        sleep(5000);
        drive.followTrajectorySequence(backToPickup2);

        sleep(2000);

        drive.followTrajectorySequence(backToDropOff2);
        sleep(2000);
        drive.followTrajectorySequence(backToPickup3);

        sleep(2000);

        drive.followTrajectorySequence(start);
        sleep(2000);
        drive.followTrajectorySequence(backToPickup1);

        while (opModeIsActive()) {
            idle();
        }
    }

    @Override
    protected void runPark() {

    }
}
