package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.LiftTele;

public class LiftDepositSpecimenCommand extends CommandBase{
    LiftTele lift;
    Telemetry telemetry;

    public LiftDepositSpecimenCommand(LiftTele lift, Telemetry telemetry) {
        this.lift = lift;
        this.telemetry = telemetry;
    }

    public void initialize() {lift.setLiftSpecimenHighBar();}

    public void execute() {

    }

    public boolean isFinished () {return lift.inTolerance();}

}


