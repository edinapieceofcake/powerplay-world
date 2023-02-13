package edu.edina.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.Stickygamepad;

@TeleOp
@Disabled
public class TestIntake extends LinearOpMode {
    private Servo clawServo;
    private Servo armServo;
    private DcMotorEx liftMotor;
    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx centerEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        Stickygamepad pad1 = new Stickygamepad(gamepad1);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        centerEncoder = hardwareMap.get(DcMotorEx.class, "centerEncoder");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the digital channel to input.
        armServo.setPosition(.5);
        clawServo.setPosition(.5);

        waitForStart();

        while (opModeIsActive()) {
            pad1.update();

            if (pad1.dpad_left) {
                clawServo.setPosition(clawServo.getPosition() + .01);
            }

            if (pad1.dpad_right) {
                clawServo.setPosition(clawServo.getPosition() - .01);
            }

            if (pad1.x) {
                armServo.setPosition(armServo.getPosition() + .01);
            }

            if (pad1.b) {
                armServo.setPosition(armServo.getPosition() - .01);
            }

            telemetry.addData("Claw Servo", clawServo.getPosition());
            telemetry.addData("Arm Servo", armServo.getPosition());
            telemetry.addData("Lift Location", liftMotor.getCurrentPosition());

            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());

            telemetry.update();
        }
    }
}
