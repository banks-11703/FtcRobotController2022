package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static final int BLUE = 0;
    public static final int RED  = 1;
    public static final int LEFT  = 0;
    public static final int RIGHT = 1;

    static int isRed = 1;
    static int isRight = RIGHT;
    static int scannerOutput = 0;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(300);


    Pose2d A1 = new Pose2d( 60, 60 * isRed, Math.toRadians(-90 * isRed));
    Pose2d A2 = new Pose2d( 60, 36 * isRed, Math.toRadians(-90 * isRed));
    Pose2d A3 = new Pose2d( 60, 12 * isRed, Math.toRadians(-90 * isRed));
    Pose2d B1 = new Pose2d( 36, 60 * isRed, Math.toRadians(-90 * isRed));
    Pose2d B2 = new Pose2d( 36, 36 * isRed, Math.toRadians(-90 * isRed));
    Pose2d B3 = new Pose2d( 36, 12 * isRed, Math.toRadians(-90 * isRed));
    Pose2d C1 = new Pose2d( 12, 60 * isRed, Math.toRadians(-90 * isRed));
    Pose2d C2 = new Pose2d( 12, 36 * isRed, Math.toRadians(-90 * isRed));
    Pose2d C3 = new Pose2d( 12, 12 * isRed, Math.toRadians(-90 * isRed));
    Pose2d D1 = new Pose2d(-12, 60 * isRed, Math.toRadians(-90 * isRed));
    Pose2d D2 = new Pose2d(-12, 36 * isRed, Math.toRadians(-90 * isRed));
    Pose2d D3 = new Pose2d(-12, 12 * isRed, Math.toRadians(-90 * isRed));
    Pose2d E1 = new Pose2d(-36, 60 * isRed, Math.toRadians(-90 * isRed));
    Pose2d E2 = new Pose2d(-36, 36 * isRed, Math.toRadians(-90 * isRed));
    Pose2d E3 = new Pose2d(-36, 12 * isRed, Math.toRadians(-90 * isRed));
    Pose2d F1 = new Pose2d(-60, 60 * isRed, Math.toRadians(-90 * isRed));
    Pose2d F2 = new Pose2d(-60, 36 * isRed, Math.toRadians(-90 * isRed));
    Pose2d F3 = new Pose2d(-60, 12 * isRed, Math.toRadians(-90 * isRed));
        RoadRunnerBotEntity myBot;





                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                switch(scannerOutput){
                    case 1:
                        myBot = new DefaultBotBuilder(meepMeep)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(calculateStartingX(), calculateStartingY(), calculateStartingRotation()))
                                                .strafeRight(24)
                                                .forward(50.6)
                                                .strafeLeft(24)
                                                .build()
                                );
                        break;
                    case 2:
                        myBot = new DefaultBotBuilder(meepMeep)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(calculateStartingX(), calculateStartingY(), calculateStartingRotation()))
                                                .strafeLeft(24)
                                                .forward(26.6)
                                                .build()
                                );
                        break;
                    default:
                        myBot = new DefaultBotBuilder(meepMeep)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(calculateStartingX(), calculateStartingY(), calculateStartingRotation()))
                                                .strafeRight(24)
                                                .forward(26.6)
                                                .build()
                                );
                        break;

                }


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();



    }
    public static double calculateStartingX(){
        if(isRed == RED){
            if(isRight == RIGHT){
                return 36;
            }else{
                return -36;
            }
        }else{
            if(isRight == RIGHT){
                return -36;
            }else{
                return 36;
            }
        }


    }
    public static double calculateStartingY(){
        if(isRed == RED){
            return -62.6;
        }else{
            return 62.6;
        }
    }
    public static double calculateStartingRotation(){
        if(isRed == RED){
            return Math.toRadians(90);
        }else{
            return Math.toRadians(-90);
        }

    }



}