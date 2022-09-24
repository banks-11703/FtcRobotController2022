//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.util.Angle;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
//
//public class MeepMeepTesting {
//    public static final int BLUE = 0;
//    public static final int RED  = 1;
//    public static final int LEFT  = 0;
//    public static final int RIGHT = 1;
//
//    static int isRed = 1;
//    static int isRight = RIGHT;
//    static int scannerOutput = 0;
//    public static void main(String[] args) {
//
//
//
//        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//
//
//        Trajectory myTrajectory;
//
//
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                switch(scannerOutput){
//                    case 1:
//                        myTrajectory = drive.trajectoryBuilder(new Pose2d())
//                                                .strafeRight(24)
//                                                .forward(50.6)
//                                                .strafeLeft(24)
//                                                .build();
//                        break;
//                    case 2:
//                        myTrajectory = drive.trajectoryBuilder(new Pose2d())
//                                                .strafeLeft(24)
//                                                .forward(26.6)
//                                                .build();
//                        break;
//                    default:
//                        myTrajectory = drive.trajectoryBuilder(new Pose2d())
//                                                .strafeRight(24)
//                                                .forward(26.6)
//                                                .build();
//                        break;
//
//                }
//
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//
//
//
//    }
//    public static double calculateStartingX(){
//        if(isRed == RED){
//            if(isRight == RIGHT){
//                return 36;
//            }else{
//                return -36;
//            }
//        }else{
//            if(isRight == RIGHT){
//                return -36;
//            }else{
//                return 36;
//            }
//        }
//
//
//    }
//    public static double calculateStartingY(){
//        if(isRed == RED){
//            return -62.6;
//        }else{
//            return 62.6;
//        }
//    }
//    public static double calculateStartingRotation(){
//        if(isRed == RED){
//            return Math.toRadians(90);
//        }else{
//            return Math.toRadians(-90);
//        }
//
//    }
//
//
//
//}
//
//
///*
//    Pose2d A1 = new Pose2d( 60 * isRed, 60 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d A2 = new Pose2d( 60 * isRed, 36 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d A3 = new Pose2d( 60 * isRed, 12 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d B1 = new Pose2d( 36 * isRed, 60 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d B2 = new Pose2d( 36 * isRed, 36 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d B3 = new Pose2d( 36 * isRed, 12 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d C1 = new Pose2d( 12 * isRed, 60 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d C2 = new Pose2d( 12 * isRed, 36 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d C3 = new Pose2d( 12 * isRed, 12 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d D1 = new Pose2d(-12 * isRed, 60 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d D2 = new Pose2d(-12 * isRed, 36 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d D3 = new Pose2d(-12 * isRed, 12 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d E1 = new Pose2d(-36 * isRed, 60 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d E2 = new Pose2d(-36 * isRed, 36 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d E3 = new Pose2d(-36 * isRed, 12 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d F1 = new Pose2d(-60 * isRed, 60 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d F2 = new Pose2d(-60 * isRed, 36 * isRed, Math.toRadians(-90 * isRed));
//    Pose2d F3 = new Pose2d(-60 * isRed, 12 * isRed, Math.toRadians(-90 * isRed));
//*/