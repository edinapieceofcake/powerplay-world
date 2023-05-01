package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

import edu.edina.library.subsystems.Claw;
import edu.edina.library.subsystems.Lift;
import edu.edina.library.subsystems.MecanumDrive;
import edu.edina.library.subsystems.Subsystem;
import edu.edina.library.util.RobotHardware;
import edu.edina.library.util.RobotState;

public class MultiThreadRobot {
    private ExecutorService subsystemUpdateExecutor;
    private boolean started;

    private List<Subsystem> subsystems;

    private Telemetry telemetry;
    public MecanumDrive driveRR;
    public Lift lift;
    public Claw claw;
    public RobotState robotState = new RobotState();
    public RobotHardware robotHardware;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                    } catch (Throwable t) {
                        this.telemetry.addData("Exception running thread 1", "");
                        this.telemetry.update();
                    }
                }
            } catch (Throwable t) {
                this.telemetry.addData("Exception running thread 2", "");
                this.telemetry.update();
            }
        }
    };

    public MultiThreadRobot(Telemetry telemetry, HardwareMap map) {
        this.telemetry = telemetry;
        this.robotHardware = new RobotHardware(map, robotState);

        subsystems = new ArrayList<>();

        try {
            driveRR = new MecanumDrive(map, robotState);
            subsystems.add(driveRR);
        } catch (IllegalArgumentException e) {

        }

        try {
            lift = new Lift(robotState, robotHardware);
            subsystems.add(lift);
        } catch (IllegalArgumentException e){

        }

        try {
            claw = new Claw(robotState, robotHardware);
            subsystems.add(lift);
        } catch (IllegalArgumentException e){

        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    public void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
            started = false;
        }
    }

    public void telemetry()
    {
        robotState.telemetry(telemetry);
    }
}
