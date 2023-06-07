package edu.edina.library.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public Servo clawTiltServo;
    public Servo leftServo;
    public Servo rightServo;
    public Servo centerServo;
    public DcMotorEx liftMotor;
    public Servo armServo;
    public Servo clawServo;
    public DigitalChannel liftSwitch;

    public RobotHardware(HardwareMap map) {
        clawTiltServo = map.get(Servo.class, "clawTiltServo");
        leftServo = map.get(Servo.class, "leftPodServo");
        rightServo = map.get(Servo.class, "rightPodServo");
        centerServo = map.get(Servo.class, "centerPodServo");
        liftMotor = map.get(DcMotorEx.class, "liftMotor");
        armServo = map.get(Servo.class, "armServo");
        clawServo = map.get(Servo.class, "clawServo");
        liftSwitch = map.get(DigitalChannel.class, "liftSwitch");

        // set the digital channel to input.
        liftSwitch.setMode(DigitalChannel.Mode.INPUT);
    }
}
