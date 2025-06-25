package org.firstinspires.ftc.teamcode.old.teleop.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorDownCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.gripperCommands.GripperOpenCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.HighBarCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftDeconflictCommand;

public class DepositSpecimenHighBarSequenceCommand extends SequentialCommandGroup {
        public DepositSpecimenHighBarSequenceCommand(BrainSTEMRobot robot, Telemetry telemetry) {
            super(
                    new HighBarCommand(robot.lift,telemetry),
                    new WaitCommand(250),
                    new GripperOpenCommand(robot.depositor,telemetry),
                    new DepositorDownCommand(robot.depositor,telemetry),
                    new LiftDeconflictCommand(robot.lift,telemetry)
            );

        }
    }
