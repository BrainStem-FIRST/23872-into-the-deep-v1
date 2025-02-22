package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.LiftTele;

public class DepositorResetAfterHighBarCommand extends CommandBase{
    LiftTele lift;
    Telemetry telemetry;

    public DepositorResetAfterHighBarCommand(LiftTele lift, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.lift = lift;
    }

    public void initialize() {
        lift.setDeconflict();
    }

    public void execute () {

    }

    public boolean isFinished() {
        return lift.inTolerance();
    }
}