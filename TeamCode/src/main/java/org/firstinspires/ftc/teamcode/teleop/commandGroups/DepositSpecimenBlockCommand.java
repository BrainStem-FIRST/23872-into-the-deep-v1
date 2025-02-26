package org.firstinspires.ftc.teamcode.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.DepositorResetAfterHighBarCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.Grabheight;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.HighBarCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftBaseCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDeconflictCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftDepositSpecimenCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftGrabSpecimenCommand;

public class DepositSpecimenBlockCommand extends SequentialCommandGroup {
    public DepositSpecimenBlockCommand(BrainSTEMRobot robot, Telemetry telemetry) {
        super(
                new LiftDeconflictCommand(robot.lift,telemetry),
                new LiftGrabCommand(robot.lift,telemetry),
                new WaitCommand(250),
                new GripperCloseCommand(robot.depositor,telemetry),
                new LiftDeconflictCommand(robot.lift,telemetry),
                new DepositorBackCommand(robot.depositor,telemetry),
                new LiftGrabSpecimenCommand(robot.lift,telemetry),
                new WaitCommand(250),
                new GripperOpenCommand(robot.depositor,telemetry),
                new LiftDeconflictCommand(robot.lift, telemetry),
                new DepositorDownCommand(robot.depositor, telemetry),
                new GripperOpenCommand(robot.depositor, telemetry)
        );

    }
}
