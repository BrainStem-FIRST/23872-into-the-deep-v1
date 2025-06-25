package org.firstinspires.ftc.teamcode.old.teleop.commandGroups;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftResetCommand;

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