
package org.firstinspires.ftc.teamcode.old.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftGrabCommand;
import org.firstinspires.ftc.teamcode.old.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorHighBasketCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftDeconflictCommand;

public class DepositGripSequenceCommand extends SequentialCommandGroup {
    public DepositGripSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry){
        super( 
                new GripperOpenCommand(robot.depositor,telemetry),
                new DepositorDownCommand(robot.depositor,telemetry),
                new LiftGrabCommand(robot.lift,telemetry),
                new WaitCommand(165),
                new GripperCloseCommand(robot.depositor,telemetry),
                new WaitCommand(250),
                new LiftDeconflictCommand(robot.lift,telemetry),
                new DepositorHighBasketCommand(robot.depositor,telemetry)
        );
    }
}