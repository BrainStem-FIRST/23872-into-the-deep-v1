
package org.firstinspires.ftc.teamcode.old.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorUpCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftDeconflictCommand;

public class DepositReleaseSequenceCommand extends SequentialCommandGroup {
    public DepositReleaseSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry){
        super(
                new GripperOpenCommand(robot.depositor, telemetry),
                new WaitCommand(250),
                new DepositorUpCommand(robot.depositor, telemetry),
                new LiftDeconflictCommand(robot.lift, telemetry),
                new DepositorDownCommand(robot.depositor, telemetry)
        );
    }
}