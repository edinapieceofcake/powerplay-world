package edu.edina.opmodes.test;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
//@Disabled
public class StrafeToColorWithColorAndDistanceRight extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor backColor = hardwareMap.get(ColorSensor.class, "backColor");
        ColorSensor frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        double startDistance = 0;
        float hsvValues[] = {0F,0F,0F};


        while (!isStarted()) {
            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0]);

            Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0]);

            startDistance = frontDistance.getDistance(DistanceUnit.INCH);
            telemetry.addLine().addData("Front", startDistance);
            telemetry.addLine().addData("Distance", leftFront.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        strafe(.2, false);

        while (!Thread.currentThread().isInterrupted()) {
            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            if ((hsvValues[0] < 100) || (hsvValues[0] > 160)){
                strafe(0, true);
                break;
            }

            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0]);

            Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0]);

            telemetry.addLine().addData("Front", frontDistance.getDistance(DistanceUnit.INCH));
            telemetry.addLine().addData("Distance", leftFront.getCurrentPosition());
            telemetry.update();
        }
/*
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        long startingPositiop = leftFront.getCurrentPosition();
        drive(.2, true);
        while (!Thread.currentThread().isInterrupted()) {
            if (Math.abs(startingPositiop - leftFront.getCurrentPosition()) > 40) {
                drive(0, true);
                break;
            }
            idle();
            telemetry.addLine().addData("Distance", leftFront.getCurrentPosition());
            telemetry.addLine().addData("Power", leftFront.getPower());
            telemetry.update();
        }
*/
        double distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);

        if (distanceToTravel > 4) {
            distanceToTravel = 4;
        }

        while (!isStopRequested()) {
            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0]);

            Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0]);

            telemetry.addLine().addData("Distance", frontDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
            idle();
        }
    }

    private void strafe(double power, boolean goLeft) {
        leftFront.setPower(goLeft ? -power : power);
        leftRear.setPower(goLeft ? power : -power);
        rightFront.setPower(goLeft ? power * .8 : -power * .8);
        rightRear.setPower(goLeft ? -power : power);
    }

    private void drive(double power, boolean goBack) {
        leftFront.setPower(goBack ? -power : power);
        leftRear.setPower(goBack ? -power : power);
        rightFront.setPower(goBack ? -power : power);
        rightRear.setPower(goBack ? -power : power);
    }
}
