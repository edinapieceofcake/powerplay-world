package edu.edina.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.RobotState;
import edu.edina.library.util.Stickygamepad;

@TeleOp
@Disabled
public class TestSlicer extends LinearOpMode {
    private Servo clawServo;
    private Servo armServo;
    private DcMotorEx liftMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        Stickygamepad pad1 = new Stickygamepad(gamepad1);
        RobotState robotState = new RobotState();

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the digital channel to input.
        armServo.setPosition(robotState.ARMFRONTPOSITION);
        clawServo.setPosition(robotState.CLAWOPENPOSITION);

        waitForStart();

        liftMotor.setPower(.5);
        while (opModeIsActive()) {
            pad1.update();

            if (pad1.dpad_left) {
                armServo.setPosition(robotState.ARMFRONTPOSITION);
            }

            if (pad1.dpad_up) {
                armServo.setPosition(robotState.ARMSIDEPOSITION);
            }

            if (pad1.dpad_right) {
                armServo.setPosition(robotState.ARMBACKPOSITION);
            }

            if (pad1.a) {
                liftMotor.setTargetPosition(0);
            }

            if (pad1.b) {
                liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH);
            }

            if (pad1.y) {
                // slicer in front
                clawServo.setPosition(robotState.CLAWOPENPOSITION);
                sleep(100);
            }

            if (pad1.x) {
                // slicer in back requiring more rotation
                clawServo.setPosition(robotState.CLAWOPENPOSITION);
                sleep(100);
            }

            if (pad1.left_bumper) {
            }

            if (pad1.right_bumper) {
            }

            if (gamepad1.left_trigger != 0) {
                clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            }

            if (gamepad1.right_trigger != 0) {
                clawServo.setPosition(robotState.CLAWOPENPOSITION);
            }

            telemetry.addData("Claw Servo", clawServo.getPosition());
            telemetry.addData("Arm Servo", armServo.getPosition());
            telemetry.addData("Lift Location", liftMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
