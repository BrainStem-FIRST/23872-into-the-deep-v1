package org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Depositor;

public class DepositorForwardCommand extends CommandBase {
    Depositor depositor;
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();



    public DepositorForwardCommand(Depositor depositor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.depositor = depositor;
    }

    @Override
    public void initialize() {
        depositor.setDepositorForward();
        timer.reset();
    }

    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > Depositor.Params.DEPOSITOR_FORWARD_TIME_MS;
    }
}
