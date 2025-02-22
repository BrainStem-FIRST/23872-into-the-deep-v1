package org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.DepositorTele;

public class GripperOpenCommand extends CommandBase {
    DepositorTele depositor;
    Telemetry telemetry;
    ElapsedTime timer = new ElapsedTime();



    public GripperOpenCommand(DepositorTele depositor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.depositor = depositor;
    }

    @Override
    public void initialize() {
        depositor.setGripperOpen();
        timer.reset();
    }

    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > DepositorTele.Params.GRIPPER_OPEN_TIME_MS;
    }
}
