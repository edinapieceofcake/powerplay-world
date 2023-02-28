package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.37795276 / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -.5; // X is the up and down direction
    public static double PARALLEL_Y = -9.9375/2; // Y is the strafe direction

    public static double PERPENDICULAR_X = 3.779;
    public static double PERPENDICULAR_Y = 0;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDriveTwoWheelOdo drive;
    public static double X_MULTIPLIER = 1.034; //1.020; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.031; //1.020; // Multiplier in the Y direction

    private List<Integer> lastEncPositions, lastEncVels;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDriveTwoWheelOdo drive, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"), "rightEncoder");
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "centerEncoder"), "centerEncoder");

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int parallelPos = parallelEncoder.getCurrentPosition();
        int perpendicularPos = perpendicularEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(parallelPos);
        lastEncPositions.add(perpendicularPos);

        return Arrays.asList(
                encoderTicksToInches(parallelPos) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        int parallelVel = (int) parallelEncoder.getCorrectedVelocity();
        int perpendicularVel = (int) perpendicularEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(parallelVel);
        lastEncVels.add(perpendicularVel);

        return Arrays.asList(
                encoderTicksToInches(parallelVel) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularVel) * Y_MULTIPLIER
        );
    }
}