//package org.firstinspires.ftc.teamcode.teleop.commands.extensionCommands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.robot.subsystems.CollectorExtension;
//
//public class ExtensionOutCommand extends CommandBase {
//    CollectorExtension collectorExtension;
//    Telemetry telemetry;
//
//
//
//    public ExtensionOutCommand(CollectorExtension collectorExtension, Telemetry telemetry) {
//        this.telemetry = telemetry;
//        this.collectorExtension = collectorExtension;
//    }
//
//    @Override
//    public void initialize() {
//        collectorExtension.setOut();
//    }
//    public void execute(){
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
//}
