package edu.edina.library.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotState {
    public DriveSpeed DriveSpeed = edu.edina.library.util.DriveSpeed.Low;

    // lateral distance 9.3125
    // offset 2.56 towards the front
    public long LiftDiff;
    public long LiftMotorLocation = 0;
    public ClawServoPosition ClawServoPosition = edu.edina.library.util.ClawServoPosition.Closed;
    public ArmServoPosition ArmServoPosition = edu.edina.library.util.ArmServoPosition.Front;
    public PoleLocation TargetPoleLocation = edu.edina.library.util.PoleLocation.None;
    public ClawRotation ClawRotation = edu.edina.library.util.ClawRotation.Center;
    public double ClawPosition = 0.0;
    public double ArmPosition = 0.0;
    public boolean LiftSwitch = false;
    public boolean LiftMotorReset = false;
    public double SpeedMultiplier = 0.5;
/*
    Original Claw - Uncomment if swapped back and comment out current settings
    public double CLAWOPENPOSITION = 0.56;
    public double CLAWMIDDLEPOSITION = 0.7;
    public double CLAWCLOSEDPOSITION = 0.83;
    public double CLAWOPENFORDROPOFF = .59;
    public int CLAWOPENPOSITION100 = 56;
*/
    public double CLAWOPENPOSITION = 0.46;
    public int CLAWOPENPOSITION100 = 46;
    public double CLAWMIDDLEPOSITION = 0.53;
    public double CLAWCLOSEDPOSITION = 0.61;
    public double CLAWOPENFORDROPOFF = .37;
    public double CLAWRIGHTPICKUPTILTPOSITION = 0.42;
    public double CLAWLEFTPICKUPTILTPOSITION = 0.62;

    public double CLAWRIGHTDROPOFFTILTPOSITION = 0.25;
    public double CLAWLEFTDROPOFFTILTPOSITION = 0.85;

    public double ARMFRONTPOSITION = 0.14;
    public double ARMBACKPOSITION = 0.83;
    public double ARMSIDEPOSITION = 0.5;

    public double CLAWCENTERTILT = 0.5;
    public double CLAWLEFTTILT = 0.73;
    public double CLAWRIGHTTILT = 0.35;

    public int CONESTACKPOSITION5 = -155;
    public int CONESTACKPOSITION4 = -115;
    public int CONESTACKPOSITION3 = -65;
    public int CONESTACKPOSITION2 = -35;
    public int CONESTACKPOSITION1 = 0;

    public int POLEPOSITIONLOW = -465;
    public int POLEPOSITIONMIDDLE = -805;
    public int POLEPOSITIONHIGH = -1110;
    public int AUTOPOLEPOSITIONMEDIUM = -740;
    public int AUTOPOLEPOSITIONHIGH = -1055;

    public int CLAWOPENWAITTIME = 250;
    public int LIFTRETURNHEiGHT = 0;
    public boolean LiftSuccessfullySetup = false;
    public boolean DriveSuccessfullySetup = false;
    public int FutureTargetPosition = 0;

    public double LowSpeedMultiplier = .5;
    public double HighSpeedMultiplier = .75;

    public double LiftUpSpeed = .9;
    public double LiftDownSpeed = .5;

    public double SERVOUPPOSITION = .5;
    public double SERVODOWNPOSITION = 1.0;

    public double Voltage = 0.0;

    public RobotState() {}

    public void telemetry(Telemetry telemetry) {
        if (LiftSuccessfullySetup) {
            telemetry.addData("Lift Position", LiftMotorLocation);
            telemetry.addData("ClawPosition", ClawPosition);
            telemetry.addData("ArmPosition", ArmPosition);
            telemetry.addData("LiftDiff", LiftDiff);
            telemetry.addData("LiftSwitch", LiftSwitch);
            telemetry.addData("LiftMotorReset", LiftMotorReset);
            telemetry.addData("Future Target Position", FutureTargetPosition);
            telemetry.addData("TargetPoleLocation", TargetPoleLocation);
            telemetry.addData("LiftUpSpeed", LiftUpSpeed);
            telemetry.addData("LiftDownSpeed", LiftDownSpeed);
            telemetry.addData("Voltage", Voltage);
        } else {
            telemetry.addData("Unable to setup motors liftMotor or setup servos armServo or latchServo", "");
        }

        if (DriveSuccessfullySetup) {
            telemetry.addData("Drive Speed", DriveSpeed);
            telemetry.addData("Speed Multiplier", SpeedMultiplier);
            telemetry.addData("LowSpeedMultiplier", LowSpeedMultiplier);
            telemetry.addData("HighSpeedMultiplier", HighSpeedMultiplier);
        } else {
            telemetry.addData("edu.edina.library.subsystems.MecanumDrive: Unable to setup frontLeft, frontRight, backLeft, backRight motors", "");
        }
    }
}
