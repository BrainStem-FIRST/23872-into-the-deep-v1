package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Lift;

import java.util.Timer;

public class LiftResetCommand extends CommandBase {
    Lift lift;
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();
    boolean isFinished = false;



    public LiftResetCommand(Lift lift, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.lift = lift;
    }

    @Override
    public void initialize() {
        lift.setReset();
    }

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
