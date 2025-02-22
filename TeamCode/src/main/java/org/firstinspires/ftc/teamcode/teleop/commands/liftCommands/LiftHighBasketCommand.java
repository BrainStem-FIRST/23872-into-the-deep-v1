package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.LiftTele;

public class LiftHighBasketCommand extends CommandBase {
    LiftTele lift;
    Telemetry telemetry;



    public LiftHighBasketCommand(LiftTele lift, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.lift = lift;
    }

    @Override
    public void initialize() {
        lift.setHighBasket();
    }

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return lift.inTolerance();
    }
}
