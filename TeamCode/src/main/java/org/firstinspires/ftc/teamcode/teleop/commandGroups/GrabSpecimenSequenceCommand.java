package org.firstinspires.ftc.teamcode.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDeconflictCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabSpecimenCommand;

public class GrabSpecimenSequenceCommand extends SequentialCommandGroup {
    public GrabSpecimenSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry) {
        super(
                new LiftDeconflictCommand(robot.lift,telemetry),
                new LiftGrabSpecimenCommand(robot.lift, telemetry),
                new DepositorBackCommand(robot.depositor, telemetry)
        );

    }
}
