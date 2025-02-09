package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.auto.subsystem.CollectorAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.ComponentAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.DepositorAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.ExtensionAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.LiftAuto;

import java.util.ArrayList;

public class BrainSTEMRobot {
    Telemetry telemetry;
    HardwareMap map;
    ArrayList<ComponentAuto> subsystems;
    public LiftAuto lift;
    public DepositorAuto depositor;
    public CollectorAuto collector;
    public ExtensionAuto extension;

    public PinpointDrive drive;



    public BrainSTEMRobot(Telemetry telemetry, HardwareMap map, Pose2d pose){
        this.telemetry = telemetry;
        this.map = map;

        subsystems = new ArrayList<>();
        lift = new LiftAuto(map, telemetry);
        depositor = new DepositorAuto(map, telemetry);
        collector = new CollectorAuto(map, telemetry);
        extension = new ExtensionAuto(map, telemetry);
        drive = new PinpointDrive(map, pose);

        subsystems.add(lift);
        subsystems.add(depositor);
        subsystems.add(collector);
        subsystems.add(extension);

    }

    public void update() {
        for (ComponentAuto c : subsystems) {
            c.update();
        }
        drive.updatePoseEstimate();
        CommandScheduler.getInstance().run();
    }
}
