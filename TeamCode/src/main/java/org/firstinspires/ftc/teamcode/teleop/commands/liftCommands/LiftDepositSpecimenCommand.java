package org.firstinspires.ftc.teamcode.teleop.commands.liftCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorUpCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDeconflictCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabCommand;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Lift;

public class LiftDepositSpecimenCommand extends CommandBase{
    Lift lift;
    Telemetry telemetry;

    public LiftDepositSpecimenCommand(Lift lift, Telemetry telemetry) {
        this.lift = lift;
        this.telemetry = telemetry;
    }

    public void initialize() {lift.setLiftSpecimenHighBar();}

    public void execute() {

    }

    public boolean isFinished () {return lift.inTolerance();}

}


