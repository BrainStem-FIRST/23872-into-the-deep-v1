package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d beginPose = new Pose2d(-39, -64, Math.toRadians(0));
        Pose2d depositPose = new Pose2d(-59, -59, Math.toRadians(50));
        Pose2d rightBlockPose = new Pose2d(-47, -49, Math.toRadians(90));
        Pose2d centerBlockPose = new Pose2d(-59, -45, Math.toRadians(92));
        Pose2d leftBlockPose = new Pose2d(-52, -32, Math.toRadians(160));
        Pose2d parkPose = new Pose2d(-20, -12, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.3)
                .setDimensions(15,15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .setReversed(true)
                        .splineToLinearHeading(depositPose, Math.toRadians(210))
                        .setReversed(false)
                        .splineToLinearHeading(rightBlockPose, Math.toRadians(75))
                        .setReversed(true)
                        .splineToLinearHeading(depositPose, Math.toRadians(210))
                        .setReversed(false)
                        .splineToLinearHeading(centerBlockPose, Math.toRadians(90))
                        .setReversed(true)
                        .splineToLinearHeading(depositPose, Math.toRadians(225))
                        .setReversed(false)
                        .splineToLinearHeading(leftBlockPose, Math.toRadians(110))
                        .setReversed(true)
                        .splineToLinearHeading(depositPose, Math.toRadians(225))
                        .setReversed(false)
                        .splineToLinearHeading(parkPose, Math.toRadians(0))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}