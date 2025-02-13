package org.firstinspires.ftc.teamcode.teleop.commandGroups;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.HighBarPreHeightCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftResetCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftSpecimenPreDeposit;

public class ResetLiftCommand extends SequentialCommandGroup {
    public ResetLiftCommand(BrainSTEMRobot robot, Telemetry telemetry) {
        super(
                new GripperOpenCommand(robot.depositor, telemetry),
                new DepositorDownCommand(robot.depositor, telemetry),
                new WaitCommand(500),
                new LiftResetCommand(robot.lift, telemetry)
        );

    }

}