package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.LiftTele;

public class HighBarCommand extends CommandBase{
    LiftTele lift;
    Telemetry telemetry;

    public HighBarCommand(LiftTele lift, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.lift = lift;
    }

    public void initialize() {
        lift.setHighBar();
    }

    public void execute () {

    }

    public boolean isFinished() {
        return lift.greaterHighBar();
    }
}



