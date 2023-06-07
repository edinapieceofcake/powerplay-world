package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.POCSampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.ThreeWheelSampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

import edu.edina.library.util.Stickygamepad;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
//@Disabled
public class LocalizationTest extends LinearOpMode {
    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 4.6875 * 2;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 3.0625;

    public static final double WHEEL_DIAMETER = 1.37795276;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 1000.0;
    public static final double GEAR_RATIO = 22.0/30.0;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER * GEAR_RATIO / TICKS_PER_REV;

    private MotorEx leftEncoder, rightEncoder, frontEncoder;

    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        Stickygamepad gamepad = new Stickygamepad(gamepad1);

        Servo leftServo = hardwareMap.get(Servo.class, "leftPodServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightPodServo");
        Servo centerServo = hardwareMap.get(Servo.class, "centerPodServo");
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        centerServo.setPosition(1);
        SampleMecanumDrive drive2 = new SampleMecanumDrive(hardwareMap);
        POCSampleMecanumDrive pocDrive = new POCSampleMecanumDrive(hardwareMap);
        ThreeWheelSampleMecanumDrive drive3 = new ThreeWheelSampleMecanumDrive(hardwareMap);
        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder =  new MotorEx(hardwareMap,  "rightEncoder");
        frontEncoder =  new MotorEx(hardwareMap,  "centerEncoder");
        TrajectorySequence strafeOut2 = drive2.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .strafeTo(new Vector2d(3, -10))
                .build();
        TrajectorySequence strafeIn2 = drive2.trajectorySequenceBuilder(strafeOut2.end())
                .strafeTo(new Vector2d(0, 0))
                .build();
        TrajectorySequence strafeOut3 = drive3.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .strafeTo(new Vector2d(3, -10))
                .build();
        TrajectorySequence strafeIn3 = drive3.trajectorySequenceBuilder(strafeOut3.end())
                .strafeTo(new Vector2d(0, 0))
                .build();


        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = leftEncoder.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        leftOdometer.reset();
        rightOdometer = rightEncoder.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer.reset();
        centerOdometer = frontEncoder.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer.reset();

        leftOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        //drive3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive2.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        drive3.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        while (!isStopRequested()) {
            drive3.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .5,
                            -gamepad1.left_stick_x * .5,
                            -gamepad1.right_stick_x *.5
                    )
            );

            drive2.update();
            drive3.update();
            pocDrive.update();
            gamepad.update();
            odometry.updatePose(); // update the position

            Pose2d poseEstimate2 = drive2.getPoseEstimate();
            telemetry.addData("RR2 x", poseEstimate2.getX());
            telemetry.addData("RR2 y", poseEstimate2.getY());
            telemetry.addData("RR2 heading", Math.toDegrees(poseEstimate2.getHeading()));

            Pose2d poseEstimate3 = drive3.getPoseEstimate();
            telemetry.addData("RR3 x", poseEstimate3.getX());
            telemetry.addData("RR3 y", poseEstimate3.getY());
            telemetry.addData("RR3 heading", Math.toDegrees(poseEstimate3.getHeading()));

            Pose2d poseEstimatePOC = pocDrive.getPoseEstimate();
            telemetry.addData("POC x", poseEstimatePOC.getX());
            telemetry.addData("POC y", poseEstimatePOC.getY());
            telemetry.addData("POC heading", Math.toDegrees(poseEstimatePOC.getHeading()));

            telemetry.addData("FTCLib x", odometry.getPose().getX());
            telemetry.addData("FTCLib y", odometry.getPose().getY());
            telemetry.addData("FTCLib heading", Math.toDegrees(odometry.getPose().getHeading()));
            telemetry.update();

            System.out.println(String.format("AllPoses(2wheel, 3wheel, youtube, openodo): %f %f %f %f %f %f %f %f %f %f %f %f",
                    poseEstimate2.getX(), poseEstimate2.getY(), poseEstimate2.getHeading(),
                    poseEstimate3.getX(), poseEstimate3.getY(), poseEstimate3.getHeading(),
                    poseEstimatePOC.getX(), poseEstimatePOC.getY(), poseEstimatePOC.getHeading(),
                    odometry.getPose().getX(), odometry.getPose().getY(), odometry.getPose().getHeading()));

            if (gamepad.a) {
                drive3.followTrajectorySequence(strafeOut3);
            }
            if (gamepad.b) {
                drive3.followTrajectorySequence(strafeIn3);
            }

            if (gamepad.dpad_left) {
                drive2.followTrajectorySequence(strafeOut2);
            }
            if (gamepad.dpad_right) {
                drive2.followTrajectorySequence(strafeIn2);
            }
        }
    }
}
