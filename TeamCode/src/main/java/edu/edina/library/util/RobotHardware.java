package edu.edina.library.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public Servo clawTiltServo;
    public DcMotorEx liftMotor;
    public Servo armServo;
    public Servo clawServo;
    public DigitalChannel liftSwitch;
    public Servo leftServo;
    public Servo rightServo;
    public Servo centerServo;

    public RobotHardware(HardwareMap map, RobotState robotState) {
        clawTiltServo = map.get(Servo.class, "clawTiltServo");
        liftMotor = map.get(DcMotorEx.class, "liftMotor");
        armServo = map.get(Servo.class, "armServo");
        clawServo = map.get(Servo.class, "clawServo");
        liftSwitch = map.get(DigitalChannel.class, "liftSwitch");

        // set the digital channel to input.
        liftSwitch.setMode(DigitalChannel.Mode.INPUT);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(robotState.LiftUpSpeed);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftServo = map.get(Servo.class, "leftPodServo");
        rightServo = map.get(Servo.class, "rightPodServo");
        centerServo = map.get(Servo.class, "centerPodServo");
    }
}
