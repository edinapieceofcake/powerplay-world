package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1000;
    public static double WHEEL_RADIUS = 1.37795276 / 2; // in
    public static double GEAR_RATIO = 22.0/30.0; // output (wheel) speed / input (encoder) speed

    //9.42, 8.49
    public static double PARALLEL_X = -0.5625; // X is the up and down direction
    public static double PARALLEL_Y = 4.6875; // Y is the strafe direction

    public static double PERPENDICULAR_X = 3.0625;
    public static double PERPENDICULAR_Y = -0.125;
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public static double X_MULTIPLIER = 0.95051; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.94529; // Multiplier in the Y direction
    private List<Integer> lastEncPositions, lastEncVels;
    SampleMecanumDrive drive;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0), // left
                new Pose2d(PARALLEL_X, -PARALLEL_Y, 0), // right
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90)) // front
        ), drive);

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;
        this.drive = drive;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "centerEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    public void resetEncoderPositions() {
        leftEncoder.stopAndResetEncoder();
        rightEncoder.stopAndResetEncoder();
        frontEncoder.stopAndResetEncoder();
    }
}