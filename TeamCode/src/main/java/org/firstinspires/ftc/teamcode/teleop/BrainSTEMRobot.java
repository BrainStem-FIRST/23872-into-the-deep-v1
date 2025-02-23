package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Collector;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Component;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Depositor;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Extension;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Lift;

import java.util.ArrayList;

public class BrainSTEMRobot {
    Telemetry telemetry;
    HardwareMap map;
    ArrayList<Component> subsystems;
    public Lift lift;
    public Depositor depositor;
    public Collector collector;
    public Extension extension;

    public PinpointDrive drive;

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap map){
        this.telemetry = telemetry;
        this.map = map;

        subsystems = new ArrayList<>();
        lift = new Lift(map, telemetry);
        depositor = new Depositor(map, telemetry);
        collector = new Collector(map, telemetry);
        extension = new Extension(map, telemetry);
        drive = new PinpointDrive(map, new Pose2d(0,0,0));

        subsystems.add(lift);
        subsystems.add(depositor);
        subsystems.add(collector);
        subsystems.add(extension);
    }

    public void update() {
        for (Component c : subsystems) {
            c.update();
        }
        drive.updatePoseEstimate();
        CommandScheduler.getInstance().run();
    }
}
