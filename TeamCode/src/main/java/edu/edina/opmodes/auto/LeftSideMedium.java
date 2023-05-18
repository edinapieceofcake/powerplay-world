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

    protected static double STACK_X = -57;
    protected static double STACK_Y = -12.5;
    protected static double D2_X = -18.75;
    protected static double D2_Y = -12.5;
    public static Vector2d D2 = new Vector2d(-31.5, -18.75);
    public static Vector2d STACK_1 = new Vector2d(STACK_X, STACK_Y);
    public static Vector2d D2_1 = new Vector2d(D2_X, D2_Y);
    public static Vector2d STACK_2 = new Vector2d(STACK_X, STACK_Y);
    public static Vector2d D2_2 = new Vector2d(D2_X, D2_Y);
    public static Vector2d STACK_3 = new Vector2d(STACK_X, STACK_Y);
    public static Vector2d D2_3 = new Vector2d(D2_X, D2_Y);
    public static Vector2d STACK_4 = new Vector2d(STACK_X, STACK_Y);
    public static Vector2d D2_4 = new Vector2d(D2_X, D2_Y);
    public static Vector2d STACK_5 = new Vector2d(STACK_X, STACK_Y);
    public static Vector2d D2_5 = new Vector2d(D2_X, D2_Y);

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

        // cone one drop off
        start = drive.trajectorySequenceBuilder(getStartPose())
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);
                })
                .addTemporalMarker(1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2.1, () -> {
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .addTemporalMarker(2.2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .strafeTo(D2)
                .build();

        // cone two pickup
        backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeRight(7)
                .strafeTo(STACK_1)
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                })
                .addTemporalMarker(2.3, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone two drop off
        backToDropOff1 = drive.trajectorySequenceBuilder(backToPickup1.end())
                .strafeTo(D2_1)
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(1.8, () -> {
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWOPENFORDROPOFF);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .build();

        // cone three pickup
        backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeTo(STACK_2)
                .addTemporalMarker(.4, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(.8, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone three drop off
        backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(D2_2)
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(1.8, () -> {
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .build();

        // cone four pickup
        backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeTo(STACK_3)
                .addTemporalMarker(.4, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(.8, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone four drop off
        backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(D2_3)
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(1.8, () -> {
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .build();

        // cone five pickup
        backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeTo(STACK_4)
                .addTemporalMarker(.4, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(.8, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone five drop off
        backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(D2_4)
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(1.8, () -> {
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .build();

        // cone six pickup
        backToPickup5 = drive.trajectorySequenceBuilder(backToDropOff4.end())
                .strafeTo(STACK_5)
                .addTemporalMarker(.4, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(.8, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone six drop off
        backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(D2_5)
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 10);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(1.8, () -> {
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .build();

        // park
        backToPickup6_left = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 40;
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
                .forward(11)
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

        System.out.println("WheelPose: drive.followTrajectorySequence(start)");
        drive.followTrajectorySequence(start);

        System.out.println("WheelPose: drive.followTrajectorySequence(backToPickup1)");
        drive.followTrajectorySequence(backToPickup1);

        //checkToMoveForward();

        System.out.println("WheelPose: drive.followTrajectorySequence(backToDropOff1)");
        drive.followTrajectorySequence(backToDropOff1);

        System.out.println("WheelPose: drive.followTrajectorySequence(backToPickup2)");
        drive.followTrajectorySequence(backToPickup2);

        //checkToMoveForward();

        System.out.println("WheelPose: drive.followTrajectorySequence(backToDropOff2)");
        drive.followTrajectorySequence(backToDropOff2);

        System.out.println("WheelPose: drive.followTrajectorySequence(backToPickup3)");
        drive.followTrajectorySequence(backToPickup3);

        //checkToMoveForward();

        System.out.println("WheelPose: drive.followTrajectorySequence(backToDropOff3)");
        drive.followTrajectorySequence(backToDropOff3);

        System.out.println("WheelPose: drive.followTrajectorySequence(backToPickup4)");
        drive.followTrajectorySequence(backToPickup4);

        //checkToMoveForward();

        System.out.println("WheelPose: drive.followTrajectorySequence(backToDropOff4)");
        drive.followTrajectorySequence(backToDropOff4);

        System.out.println("WheelPose: drive.followTrajectorySequence(backToPickup5)");
        drive.followTrajectorySequence(backToPickup5);

        //checkToMoveForward();

        System.out.println("WheelPose: drive.followTrajectorySequence(backToDropOff5)");
        drive.followTrajectorySequence(backToDropOff5);

    }}
