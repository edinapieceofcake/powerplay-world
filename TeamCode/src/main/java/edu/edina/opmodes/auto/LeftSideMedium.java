package edu.edina.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.RobotState;
import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous(group = "Left")
@Config
public class LeftSideMedium extends AutoBase {

    @Override
    protected String getCameraName() {
        return "lowCamera";
    }

    @Override
    protected void addAdditionalTelemetry(Telemetry telemetry) {
        telemetry.addData("Make sure claw is in the back and low camera is facing field.", "");
        telemetry.addData("Cone should always be on side with medium pole", "");
    }

    @Override
    protected boolean shouldClawBeInTheFront() {
        return false;
    }

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(-33, -65, Math.toRadians(-180));
    }

    @Override
    protected void initPaths() {
        Vector2d startEndPoint = new Vector2d(-31.5, -19);
        //cone2
        Vector2d backToPickup1EndPoint = new Vector2d(-55.5, -11);
        Vector2d backToDropoff1EndPoint = new Vector2d(-19.5, -13);
        //cone3
        Vector2d backToPickup2EndPoint = new Vector2d(-55.5, -11);
        Vector2d backToDropoff2EndPoint = new Vector2d(-19.5, -13);
        //cone4
        Vector2d backToPickup3EndPoint = new Vector2d(-55, -13);
        Vector2d backToDropoff3EndPoint = new Vector2d(-19.5, -13);
        //cone5
        Vector2d backToPickup4EndPoint = new Vector2d(-55, -13);
        Vector2d backToDropoff4EndPoint = new Vector2d(-19, -13);
        //cone6
        Vector2d backtoPickup5EndPoint = new Vector2d(-55, -13);
        Vector2d backToDropoff5EndPoint = new Vector2d(-19, -13.5);

        // cone one drop off
        start = drive.trajectorySequenceBuilder(getStartPose())
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);
                })
                .addTemporalMarker(1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 10);
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.3, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .strafeTo(startEndPoint)
                .build();

        // cone two pickup
        backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeRight(9)
                .strafeTo(backToPickup1EndPoint)
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                })
                .build();

        // cone two drop off
        backToDropOff1 = drive.trajectorySequenceBuilder(backToPickup1.end())
                .strafeTo(backToDropoff1EndPoint)
                .addTemporalMarker(0.0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 20);
                })
                .addTemporalMarker(1.3, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENFORDROPOFF);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone three pickup
        backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeTo(backToPickup2EndPoint)
                .addTemporalMarker(.7, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                })
                .build();

        // cone three drop off
        backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(backToDropoff2EndPoint)
                .addTemporalMarker(0.0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 20);
                })
                .addTemporalMarker(1.3, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone four pickup
        backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeTo(backToPickup3EndPoint)
                .addTemporalMarker(.7, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                })
                .build();

        // cone four drop off
        backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(backToDropoff3EndPoint)
                .addTemporalMarker(0.0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 30);
                })
                .addTemporalMarker(1.3, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone five pickup
        backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeTo(backToPickup4EndPoint)
                .addTemporalMarker(.7, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                })
                .build();

        // cone five drop off
        backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(backToDropoff4EndPoint)
                .addTemporalMarker(0.0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 30);
                })
                .addTemporalMarker(1.3, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone six pickup
        backToPickup5 = drive.trajectorySequenceBuilder(backToDropOff4.end())
                .strafeTo(backtoPickup5EndPoint)
                .addTemporalMarker(.7, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .build();

        // cone six drop off
        backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(backToDropoff5EndPoint)
                .addTemporalMarker(0.0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 30);
                })
                .addTemporalMarker(1.3, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // park
        backToPickup6_left = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 50;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 60;
                    }
                })
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .forward(35)
                .build();

        backToPickup6_middle = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .forward(9)
                .build();

        backToPickup6_right = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(13)
                .build();
    }

    @Override
    protected void runPaths() {

        drive.setPoseEstimate(getStartPose());

        // cone 1
        drive.followTrajectorySequence(start);

        drive.followTrajectorySequence(backToPickup1);

        checkToMoveForward(500);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(300);

        // cone 2
        drive.followTrajectorySequence(backToDropOff1);

        drive.followTrajectorySequence(backToPickup2);

        checkToMoveForward(500);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(200);

        // cone 3
        drive.followTrajectorySequence(backToDropOff2);

        drive.followTrajectorySequence(backToPickup3);

        checkToMoveForward(500);

        sleep(100);
        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(200);

        // cone 4
        drive.followTrajectorySequence(backToDropOff3);

        drive.followTrajectorySequence(backToPickup4);

        checkToMoveForward(500);

        sleep(100);
        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(200);

        // cone 5
        drive.followTrajectorySequence(backToDropOff4);

        drive.followTrajectorySequence(backToPickup5);

        checkToMoveForward(500);

        sleep(100);
        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(200);

        // cone 6
        drive.followTrajectorySequence(backToDropOff5);
    }
}
