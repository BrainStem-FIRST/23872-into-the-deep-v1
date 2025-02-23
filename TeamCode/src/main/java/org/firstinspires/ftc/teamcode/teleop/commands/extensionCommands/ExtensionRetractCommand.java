package org.firstinspires.ftc.teamcode.teleop.commands.extensionCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Extension;

public class ExtensionRetractCommand extends CommandBase {
    Extension extension;
    Telemetry telemetry;


    public ExtensionRetractCommand(Extension collectorExtension, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.extension = extension;
    }

    @Override
    public void initialize() {
        extension.setRetract();
    }
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return extension.inTolerance();
    }
}
