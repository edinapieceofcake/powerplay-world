package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.Stickygamepad;

@TeleOp(name = "DriveMeRR", group = "teleop")
public class NoThreadTeleopRR extends OpMode {
    private NoThreadRobotRR robot;
    private Stickygamepad _gamepad1;
    private Stickygamepad _gamepad2;


    public void init() {
        _gamepad1 = new Stickygamepad(gamepad1);
        _gamepad2 = new Stickygamepad(gamepad2);

        robot = new NoThreadRobotRR(this, telemetry);

        Servo leftServo = hardwareMap.get(Servo.class, "leftPodServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightPodServo");
        Servo centerServo = hardwareMap.get(Servo.class, "centerPodServo");
        leftServo.setPosition(robot.robotState.SERVOUPPOSITION);
        rightServo.setPosition(robot.robotState.SERVOUPPOSITION);
        centerServo.setPosition(robot.robotState.SERVOUPPOSITION);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        _gamepad1.update();
        _gamepad2.update();

        // set things into the robot from the gamepad or other sensors

        robot.driveRR.setDriveProperties(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, _gamepad1.dpad_down);

        robot.lift.setLiftProperties(gamepad1.left_trigger, gamepad1.right_trigger,
                _gamepad1.dpad_left, _gamepad1.dpad_up, _gamepad1.dpad_right,
                _gamepad1.left_bumper, _gamepad1.right_bumper,
                _gamepad1.x, _gamepad1.y, _gamepad1.b, _gamepad1.a);

        robot.update();

        robot.telemetry();
    }

    @Override
    public  void stop() {

    }
}
