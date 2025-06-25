package org.firstinspires.ftc.teamcode.old.teleop.commandGroups;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftSpecimenPreDeposit;
import org.firstinspires.ftc.teamcode.old.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorForwardCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.gripperCommands.GripperCloseCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.HighBarPreHeightCommand;

public class SpecimenPreDeposit extends SequentialCommandGroup {
    public SpecimenPreDeposit(BrainSTEMRobot robot, Telemetry telemetry) {
        super(
                new GripperCloseCommand(robot.depositor, telemetry),
                new LiftSpecimenPreDeposit(robot.lift, telemetry),
                new DepositorForwardCommand(robot.depositor, telemetry),
                new HighBarPreHeightCommand(robot.lift,telemetry)
        );

    }

}