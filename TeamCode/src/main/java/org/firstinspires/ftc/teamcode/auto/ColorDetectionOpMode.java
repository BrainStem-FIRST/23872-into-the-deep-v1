package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorDetectionOpMode extends LinearOpMode {

    // Declare the color sensor
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Get the RGB values from the color sensor
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Log the RGB values for debugging
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();

            // Define the logic for detecting yellow color
            if (isYellow(red, green, blue)) {
                telemetry.addData("Color Detected", "Yellow");
            } else {
                telemetry.addData("Color Detected", "Not Yellow");
            }

            // Update telemetry
            telemetry.update();
        }
    }

    // Method to determine if the detected color is yellow
    private boolean isYellow(int red, int green, int blue) {
        return (red > 100 && green > 100 && blue < 50);
    }
}
