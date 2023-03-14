package edu.edina.opmodes.test;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class StrafeToColorWithDistanceNoRR extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        double distanceToTravel;
        double startDistance;

        while (!isStarted()) {
            distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);

            telemetry.addData("DistanceToTravel", distanceToTravel);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        startDistance = frontDistance.getDistance(DistanceUnit.INCH);

        strafe(.25, false);
        while (!Thread.currentThread().isInterrupted() && !isStopRequested()) {
            distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);

            if ((distanceToTravel - startDistance) < 5) {
                strafe(0, false);
                break;
            }
            telemetry.addData("StartDistance", startDistance);
            telemetry.addData("DistanceToTravel", distanceToTravel);
            telemetry.update();
            idle();
        }

        while (!Thread.currentThread().isInterrupted() && !isStopRequested()) {
            distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);

            telemetry.addData("Finished", "");
            telemetry.addData("StartDistance", startDistance);
            telemetry.addData("DistanceToTravel", distanceToTravel);
            telemetry.update();
            idle();
        }
    }

    private void strafe(double power, boolean goLeft) {
        leftFront.setPower(power);
        leftRear.setPower(-power);

        rightFront.setPower(power);
        rightRear.setPower(-power);
    }
}
