package org.firstinspires.ftc.teamcode.purepursuit;//package org.firstinspires.ftc.teamcode.robot.purepursuit;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import java.util.ArrayList;
//
//public class TestPath extends OpMode {
//    RobotMovement robotMovement;
//
//    @Override
//    public void init() {
//        robotMovement = new RobotMovement(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0,0,1.0,1.0,50, Math.toRadians(50), 1.0));
//        allPoints.add(new CurvePoint(50,50,1.0,1.0,50, Math.toRadians(50), 1.0));
//        allPoints.add(new CurvePoint(100,100,1.0,1.0,50, Math.toRadians(50), 1.0));
//
//        robotMovement.followCurve(allPoints, Math.toRadians(90));
//    }
//}
