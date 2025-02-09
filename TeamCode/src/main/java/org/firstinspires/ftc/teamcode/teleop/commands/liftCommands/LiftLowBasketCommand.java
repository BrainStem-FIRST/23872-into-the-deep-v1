package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.LiftTele;

public class LiftLowBasketCommand extends CommandBase {
    LiftTele lift;
    Telemetry telemetry;



    public LiftLowBasketCommand(LiftTele lift, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.lift = lift;
    }

    @Override
    public void initialize() {
        lift.setLowBasket();
    }

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return lift.inTolerance();
    }
}
