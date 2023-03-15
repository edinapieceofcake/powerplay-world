package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.sql.Time;

import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.PoleLocation;
import edu.edina.library.util.RobotState;
import edu.edina.library.util.ClawRotation;
import edu.edina.library.util.Stickygamepad;

public class Claw extends edu.edina.library.subsystems.Subsystem {
    private Stickygamepad _gamepad2;
    private Servo clawRotationServo;

    public Claw(HardwareMap map, RobotState robotState) {
        clawRotationServo = map.get(Servo.class, "clawRotationServo");

    }

    @Override
    public void update() {

    }
}
