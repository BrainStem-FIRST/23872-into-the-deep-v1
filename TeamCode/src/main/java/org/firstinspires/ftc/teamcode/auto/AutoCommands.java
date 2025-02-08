package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;
    public Pose2d localizationPose;
    public double finalHeadingPose = 0;


    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot; 
        this.telemetry = telemetry;


    }

    public void updateDrivetrain(double xPower, double yPower, double turnPower){
        double speedFactor = 1.0;
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        xPower * speedFactor,
                        yPower * speedFactor

                ),
                turnPower * speedFactor
        ));
    }

    public Action addTelemetry(String key, Object data) {
        return new AddTelemetry(key, data);
    }

    public class AddTelemetry implements Action {
        String key;
        Object data;

        public AddTelemetry(String key, Object data) {
            this.key = key;
            this.data = data;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            telemetry.addData(key, data);
            telemetry.update();

            return false;
        }
    }

    /** STEMPilot **/

    public Action driveToPoint(double targetX, double targetY, double targetHeading, double toleranceAxial, double toleranceLateral, double toleranceHeading, boolean stopAtEnd, double timeout) {
        return new DriveToPoint(targetX, targetY, targetHeading, toleranceAxial,toleranceLateral, toleranceHeading, stopAtEnd,timeout);
    }

    public class DriveToPoint implements Action {
        double toleranceAxial, toleranceLateral, toleranceHeading;
        double targetX, targetY, targetHeading, timeout;
        boolean stopAtEnd;

        PIDController axialController = new PIDController(0.35, 0.0, 0.0);
        PIDController lateralController = new PIDController(0.35, 0.0, 0.0);
        PIDController headingController = new PIDController(0.65, 0.0, 0.0);

        ElapsedTime timer = new ElapsedTime();  // Persistent timer

        public DriveToPoint(double targetX, double targetY, double targetHeading,
                            double toleranceAxial, double toleranceLateral,
                            double toleranceHeading, boolean stopAtEnd, double timeout) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetHeading = targetHeading;
            this.toleranceAxial = toleranceAxial;
            this.toleranceLateral = toleranceLateral;
            this.toleranceHeading = toleranceHeading;
            this.stopAtEnd = stopAtEnd;
            this.timeout = timeout;

            axialController.setInputBounds(-72, 72);
            lateralController.setInputBounds(-72, 72);
            headingController.setInputBounds(-2 * Math.PI, 2 * Math.PI);

            axialController.setOutputBounds(-1, 1);
            lateralController.setOutputBounds(-1, 1);
            headingController.setOutputBounds(-1, 1);

            timer.reset();  // Start timer once
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robot.update();

            // Get current field position and heading:
            double worldXPosition = robot.drive.pinpoint.getPositionRR().position.x; // forward position
            double worldYPosition = robot.drive.pinpoint.getPositionRR().position.y; // lateral position
            double worldAngle_rad = robot.drive.pinpoint.getHeading();

            double forwardError = targetX - worldXPosition;   // x error (forward)
            double lateralError = targetY - worldYPosition;     // y error (lateral)
            double distanceError = Math.hypot(forwardError, lateralError);
            double distanceTolerance = Math.hypot(toleranceAxial, toleranceLateral);
            double robotVelocity = Math.hypot(robot.drive.pinpoint.getVelocityRR().linearVel.x, robot.drive.pinpoint.getVelocityRR().linearVel.y);

            // --- Correct PID Setup for Field Errors ---
            // In our Road Runner coordinate system, x is forward and y is lateral.
            // So the "forward" (axial) PID should work on x and the "lateral" PID on y.
            axialController.setTarget(targetX);     // forward target
            lateralController.setTarget(targetY);     // lateral target

            // Compute PID outputs (error = target - measurement):
            double forwardOutput = -axialController.update(worldXPosition);    // error in x
            double lateralOutput = -lateralController.update(worldYPosition);    // error in y

            // (Adjust these gains or the sign if your motors are reversed.)
            // If your robot's hardware requires it, you might need to flip the sign on one axis.
            // For now, we assume positive forwardOutput means "drive forward" and positive lateralOutput means "strafe right".

            // --- Heading Control (unchanged except for sign considerations) ---
            double normalizedHeading = normalizeAngle(worldAngle_rad);
            double normalizedTargetHeading = normalizeAngle(targetHeading);
            headingController.setTarget(normalizedTargetHeading);
            // Depending on your drivetrain, you may or may not need a negative sign here.
            double turnPower = -headingController.update(normalizedHeading);

            // --- Transform Field PID Outputs into Robot-Relative Commands ---
            // Rotate by -worldAngle_rad:
            double sin = Math.sin(worldAngle_rad);
            double cos = Math.cos(worldAngle_rad);
            double robotForward = forwardOutput * cos + lateralOutput * sin;
            double robotStrafe = -forwardOutput * sin + lateralOutput * cos;

            // Now send these commands to the drivetrain.
            // Note: updateDrivetrain expects (forward, strafe, turn)
            double brakingScale = 0.1;

            if (stopAtEnd && robotVelocity > 35 && distanceError < 24) {
                robotForward *= brakingScale;
                telemetry.addData("Braking Factor", true);
            } else {
                telemetry.addData("Braking Factor", false);
            }

            updateDrivetrain(robotForward, robotStrafe, turnPower);

            // --- Determine if we are still running ---
            boolean isRunning = distanceError > distanceTolerance ||
                    (Math.abs(normalizeAngle(targetHeading - worldAngle_rad)) > toleranceHeading);

            // --- Telemetry (for debugging) ---
            telemetry.addData("isRunning", isRunning);
            telemetry.addData("robotVelocity", robotVelocity);
            telemetry.addData("distanceError", distanceError);
            telemetry.addData("Target X (forward)", targetX);
            telemetry.addData("Target Y (lateral)", targetY);
            telemetry.addData("Pos X (forward)", worldXPosition);
            telemetry.addData("Pos Y (lateral)", worldYPosition);
            telemetry.addData("Heading", worldAngle_rad);
            telemetry.addData("robotForward", robotForward);
            telemetry.addData("robotStrafe", robotStrafe);
            telemetry.addData("turnPower", turnPower);
            telemetry.update();

            if (timeout > 0 && timer.seconds() > timeout) {
                isRunning = false;
            }

            if (!isRunning && stopAtEnd) {
                updateDrivetrain(0, 0, 0);
            }

            return isRunning;
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
