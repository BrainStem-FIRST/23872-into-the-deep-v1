package org.firstinspires.ftc.teamcode.purepursuit;

import static org.firstinspires.ftc.teamcode.purepursuit.MathFunction.AngleWrap;
import static org.firstinspires.ftc.teamcode.purepursuit.MathFunction.lineCircleIntersection;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {
    private double xPower;
    private double yPower;
    private double turnPower;
    private GoBildaPinpointDriverRR odo;

    public RobotMovement(GoBildaPinpointDriverRR odo) {
       this.odo = odo;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        Point robotLocation = robotLocation();
        CurvePoint followMe = getFollowPointPath(allPoints, robotLocation.x, robotLocation.y, allPoints.get(0).followDistance);

        gotoPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double xPos,
                                                double yPos, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        Point robotLocation = robotLocation();
        double robotHeading = robotHeading();

        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius,
                    startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;

            for(Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - robotLocation.y, thisIntersection.x - robotLocation.x);
                double deltaAngle = Math.abs(MathFunction.AngleWrap(angle - robotHeading));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

        return followMe;
    }

    private double robotHeading() {
        return odo.getHeading();
    }

    private Point robotLocation() {
        return new Point(odo.getPosX(), odo.getPosY());
    }

    public void gotoPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        Pose2D pos = odo.getPosition();
        double worldXPosition = pos.getX(DistanceUnit.INCH);
        double worldYPosition = pos.getY(DistanceUnit.INCH);
        double worldAngle_rad = pos.getHeading(AngleUnit.RADIANS);
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2(x=worldXPosition, y=worldYPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        xPower = (relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint))) * movementSpeed;
        yPower = (relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint))) * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        turnPower = -Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 5) {
            turnPower = 0;
        }
    }

    public double getXPower() {
        return xPower;
    }

    public double getYPower() {
        return yPower;
    }

    public double getTurnPower() {
        return turnPower;
    }
}
