package org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Depositor;

public class DepositorDownCommand extends CommandBase {
    Depositor depositor;
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();



    public DepositorDownCommand(Depositor depositor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.depositor = depositor;
    }

    @Override
    public void initialize() {
        depositor.setDepositorDown();
        timer.reset();
    }

    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > Depositor.Params.DEPOSITOR_BACK_TIME_MS;
    }
}
