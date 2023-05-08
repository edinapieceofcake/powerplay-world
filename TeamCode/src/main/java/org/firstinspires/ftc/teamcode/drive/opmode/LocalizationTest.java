package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.POCTwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

import java.util.ArrayList;
import java.util.List;

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
    private int prev_left_encoder_pos = 0;
    private int prev_right_encoder_pos = 0;
    private int prev_center_encoder_pos = 0;
    private double heading = 0;
    private double x_pos = 0;
    private double y_pos = 0;

    public static double PARALLEL_X = -0.5625; // X is the up and down direction
    public static double PARALLEL_Y = -4.6875; // Y is the strafe direction

    public static double PERPENDICULAR_X = 2.9875;
    public static double PERPENDICULAR_Y = -0.125;

    DecompositionSolver forwardSolver;
    Pose2d _poseEstimate = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftServo = hardwareMap.get(Servo.class, "leftPodServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightPodServo");
        Servo centerServo = hardwareMap.get(Servo.class, "centerPodServo");
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        centerServo.setPosition(1);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        MecanumDrive.MecanumLocalizer mecanumLocalizer = new MecanumDrive.MecanumLocalizer(drive);
        TwoWheelTrackingLocalizer twoWheelTrackingLocalizer1 = new TwoWheelTrackingLocalizer(hardwareMap, drive, lastTrackingEncPositions, lastTrackingEncVels,
                -0.5625, -4.6875, 0, 2.9175, -.125, 90);
        TwoWheelTrackingLocalizer twoWheelTrackingLocalizer2 = new TwoWheelTrackingLocalizer(hardwareMap, drive, lastTrackingEncPositions, lastTrackingEncVels,
                -0.5625, -4.6875, 0, 2.9175, -.125, 93);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        localizer.leftEncoder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localizer.rightEncoder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        localizer.frontEncoder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .5,
                            -gamepad1.left_stick_x * .5,
                            -gamepad1.right_stick_x *.5
                    )
            );

            drive.update();
//            localizer.update();
            mecanumLocalizer.update();
            twoWheelTrackingLocalizer1.update();
            twoWheelTrackingLocalizer2.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("2 Wheel Localizer", "");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            poseEstimate = twoWheelTrackingLocalizer1.getPoseEstimate();
            telemetry.addData("2 Wheel-2 Localizer 90", "");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            poseEstimate = twoWheelTrackingLocalizer2.getPoseEstimate();
            telemetry.addData("2 Wheel-3 Localizer -90", "");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            /*
            poseEstimate = localizer.getPoseEstimate();
            telemetry.addData("3 Wheel Localizer", "");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
 */
            poseEstimate = mecanumLocalizer.getPoseEstimate();
            telemetry.addData("Wheel Localizer", "");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
/*
            calculate(localizer.leftEncoder.motor.getCurrentPosition(), localizer.rightEncoder.motor.getCurrentPosition(), localizer.frontEncoder.motor.getCurrentPosition());
            telemetry.addData("Home Grown Localizer", "");
            telemetry.addData("x", x_pos);
            telemetry.addData("y", y_pos);
            telemetry.addData("heading", heading);
 */
            telemetry.addData("left", localizer.leftEncoder.motor.getCurrentPosition());
            telemetry.addData("right", localizer.rightEncoder.motor.getCurrentPosition());
            telemetry.addData("center", localizer.frontEncoder.motor.getCurrentPosition());
            telemetry.update();

            Log.i("2Wheel Encoders: ", String.format("%f\t%f\t%f", twoWheelTrackingLocalizer1.getWheelPositions().get(0), twoWheelTrackingLocalizer1.getWheelPositions().get(1), twoWheelTrackingLocalizer1.getHeading()));
            Log.i("2Wheel Velocities: ", String.format("%f\t%f\t%f", twoWheelTrackingLocalizer1.getWheelVelocities().get(0), twoWheelTrackingLocalizer1.getWheelVelocities().get(1), twoWheelTrackingLocalizer1.getHeadingVelocity()));
            Log.i("2Wheel Pose: ", String.format("%f\t%f\t%f", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading())));
/*
            Log.i("3Wheel Encoders: ", String.format("%f\t%f\t%f", localizer.getWheelPositions().get(0), localizer.getWheelPositions().get(1), localizer.getWheelPositions().get(2)));
            Log.i("3Wheel Velocities: ", String.format("%f\t%f\t%f", localizer.getWheelVelocities().get(0), localizer.getWheelVelocities().get(1), localizer.getWheelVelocities().get(2)));
            Log.i("3Wheel Pose: ", String.format("%f\t%f\t%f", localizer.getPoseEstimate().getX(), localizer.getPoseEstimate().getY(), Math.toDegrees(localizer.getPoseEstimate().getHeading())));

            Log.i("Mecanum Encoders: ", String.format("%f\t%f\t%f\t%f", drive.getWheelPositions().get(0), drive.getWheelPositions().get(1), drive.getWheelPositions().get(2), drive.getWheelPositions().get(3)));
            Log.i("Mecanum Velocities: ", String.format("%f\t%f\t%f\t%f", drive.getWheelVelocities().get(0), drive.getWheelVelocities().get(1), drive.getWheelVelocities().get(2), drive.getWheelVelocities().get(3)));
            Log.i("Mecanum Pose: ", String.format("%f\t%f\t%f", mecanumLocalizer.getPoseEstimate().getX(), mecanumLocalizer.getPoseEstimate().getY(), Math.toDegrees(mecanumLocalizer.getPoseEstimate().getHeading())));

 */
        }
    }

    private void calculate(int left_encoder_pos, int right_encoder_pos, int center_encoder_pos) {
        int delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        int delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        int delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        double phi = (delta_left_encoder_pos - delta_right_encoder_pos) / DriveConstants.TRACK_WIDTH;
        double delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        double delta_perp_pos = delta_center_encoder_pos - 2.9875 * phi;

        double delta_x = delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
        double delta_y = delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);

        x_pos += TwoWheelTrackingLocalizer.encoderTicksToInches(delta_x) * TwoWheelTrackingLocalizer.X_MULTIPLIER;
        y_pos += TwoWheelTrackingLocalizer.encoderTicksToInches(delta_y) * TwoWheelTrackingLocalizer.Y_MULTIPLIER;
        heading += phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;
    }

    private Pose2d calculate2Wheel(List<Double> deltas) {
        RealMatrix matrix = MatrixUtils.createRealMatrix(3, 1);
        matrix.setEntry(0, 0, deltas.get(0));
        matrix.setEntry(1, 0, deltas.get(1));
        matrix.setEntry(2, 0, deltas.get(2));
        RealMatrix result = forwardSolver.solve(matrix);

        Pose2d pose = new Pose2d(
                result.getEntry(0, 0),
                result.getEntry(1, 0),
                result.getEntry(2, 0));

        return Kinematics.relativeOdometryUpdate(_poseEstimate, pose);
    }

    private void buildSolver() {
        Array2DRowRealMatrix matrix = new Array2DRowRealMatrix(3, 3);

        Pose2d parallel = new Pose2d(PARALLEL_X, PARALLEL_Y, 0);
        Pose2d perpendicular = new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90));

        matrix.setEntry(0, 0, parallel.headingVec().getX());
        matrix.setEntry(0, 1, parallel.headingVec().getY());
        matrix.setEntry(0, 2, parallel.vec().getX() * parallel.headingVec().getY() - parallel.vec().getY() * parallel.headingVec().getX());
        matrix.setEntry(1, 0, perpendicular.headingVec().getX());
        matrix.setEntry(1, 1, perpendicular.headingVec().getX());
        matrix.setEntry(1, 2, perpendicular.vec().getX() * perpendicular.headingVec().getY() - perpendicular.vec().getY() * perpendicular.headingVec().getX());
        matrix.setEntry(2, 2, 1.0);

        forwardSolver = new LUDecomposition(matrix).getSolver();
    }
}
