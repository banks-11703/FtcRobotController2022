package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config

@Autonomous
public class Auto2 extends AutoCommon {

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initialization();

        waitForStart();
        timeStampStart = runtime.time();
        if(Side() == LEFT) {
            turntableMod = -1;
        } else {
            turntableMod = 1;
        }

        starting();

        if (Mode() == 1) {//doing nothing
            doNothing();
        }
        else if (Mode() == 1) {//cycling
            moveToScore();

            Trajectory Score0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(scorePos)
                    .build();
            drive.followTrajectory(Score0);
            moveLift(1600);
            sleep(1200);

            if(Side() == LEFT) {
                turnTable(-735);
            }else {
                turnTable(735);
            }
            sleep(400);

            moveLift(2400);
            sleep(300);

            openClaw();
            sleep(150);

            turnTable(0);
            moveLift(650);

            Trajectory Intake1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            drive.followTrajectory(Intake1);

            moveLift(318);
            sleep(300);

            closeClaw();
            sleep(600);

            moveLift(700);
            sleep(400);

            Trajectory Score1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(scorePos.plus(new Pose2d(0,1,Math.toRadians(0))))
                    .build();
            drive.followTrajectory(Score1);

            moveLift(1600);
            sleep(700);

            if(Side() == LEFT) {
                turnTable(-735);
            }else {
                turnTable(735);
            }
            sleep(400);

            moveLift(2400);
            sleep(300);

            openClaw();
            sleep(150);

            turnTable(0);
            moveLift(650);

            Trajectory Intake2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            drive.followTrajectory(Intake2);

            moveLift(150);
            sleep(300);

            closeClaw();
            sleep(600);

            moveLift(700);
            sleep(400);

            Trajectory Score2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(scorePos.plus(new Pose2d(0,2,Math.toRadians(0))))
                    .build();
            drive.followTrajectory(Score2);

            moveLift(1600);
            sleep(700);

            if(Side() == LEFT) {
                turnTable(-735);
            }else {
                turnTable(735);
            }
            sleep(400);

            moveLift(2400);
            sleep(300);

            openClaw();
            sleep(150);

            turnTable(0);
            moveLift(0);

            switch (autoParkPosition) {
                case 1:
                    Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(park2)
                            .build();
                    drive.followTrajectory(CyclePark2);
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (Side() == 0) {
                        Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park3)
                                .build();
                        drive.followTrajectory(CyclePark3);
                    } else {
                        Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park1)
                                .build();
                        drive.followTrajectory(CyclePark1);
                    }

                    if (isStopRequested()) return;
                    break;
                default:
                    if (Side() == 0) {
                        Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park1)
                                .build();
                        drive.followTrajectory(CyclePark1);
                    } else {
                        Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park3)
                                .build();
                        drive.followTrajectory(CyclePark3);
                    }
                    if (isStopRequested()) return;
                    break;
            }
        }
        else if (Mode() == 2) {//Just parking
            moveToScore();
            switch (autoParkPosition) {
                case 1:

                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (Side() == 0) {
                        Trajectory Park3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park3)
                                .build();
                        drive.followTrajectory(Park3);
                    } else {
                        Trajectory Park1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park1)
                                .build();
                        drive.followTrajectory(Park1);
                    }

                    if (isStopRequested()) return;
                    break;
                default:
                    if (Side() == 0) {
                        Trajectory Park1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park1)
                                .build();
                        drive.followTrajectory(Park1);
                    } else {
                        Trajectory Park3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park3)
                                .build();
                        drive.followTrajectory(Park3);
                    }
                    if (isStopRequested()) return;
                    break;
            }
        }
        else if(Mode() == 3) {//shootout
            drive.setPoseEstimate(PoseStorage.currentPose);
//            moveToScore();

            while (opModeIsActive() && !isStopRequested() && !armDone && TimeSinceStart() <= 27) {
                doLiftTasks();
                doShootoutTasks();
            }
            closeClaw();
            moveShootout(0);
            openShooterClaw();
            liftShooterClaw(false,0);

            //Park
//            drive.setPoseEstimate(PoseStorage.currentPose);
//            switch (autoParkPosition) {
//                case 1:
//                    if (Side() == RIGHT && Team() == BLUE) {
//                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(park2)
//                                .build();
//                        drive.followTrajectory(CyclePark2);
//                    } else if(Side() == LEFT && Team() == RED) {
//                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-36.5,-13.5,Math.toRadians(0)))
//                                .build();
//                        drive.followTrajectory(CyclePark2);
//                    } else if(Side() == RIGHT && Team() == RED) {
//                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(36.5,-13.5,Math.toRadians(180)))
//                                .build();
//                        drive.followTrajectory(CyclePark2);
//                    } else if(Side() == LEFT && Team() == BLUE) {
//                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(36.5,13.5,Math.toRadians(0)))
//                                .build();
//                        drive.followTrajectory(CyclePark2);
//                    }
//                    if (isStopRequested()) return;
//                    break;
//                case 2:
//                    if (Side() == 0) {
//                        if (Side() == RIGHT && Team() == BLUE) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(park3)
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        } else if(Side() == LEFT && Team() == RED) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(-11.5,-13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        } else if(Side() == RIGHT && Team() == RED) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(11.5,-13.5,Math.toRadians(180)))
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        } else if(Side() == LEFT && Team() == BLUE) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(11.5,13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        }
//                    } else {
//                        if (Side() == RIGHT && Team() == BLUE) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(park1)
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        } else if(Side() == LEFT && Team() == RED) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(-59.5,-13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        } else if(Side() == RIGHT && Team() == RED) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(59.5,-13.5,Math.toRadians(180)))
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        } else if(Side() == LEFT && Team() == BLUE) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(59.5,13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        }
//                    }
//
//                    if (isStopRequested()) return;
//                    break;
//                default:
//                    if (Side() == 0) {
//                        if (Side() == RIGHT && Team() == BLUE) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(park1)
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        } else if(Side() == LEFT && Team() == RED) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(-59.5,-13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        } else if(Side() == RIGHT && Team() == RED) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(59.5,-13.5,Math.toRadians(180)))
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        } else if(Side() == LEFT && Team() == BLUE) {
//                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(59.5,13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark1);
//                        }
//                    } else {
//                        if (Side() == RIGHT && Team() == BLUE) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(park3)
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        } else if(Side() == LEFT && Team() == RED) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(-11.5,-13.5,Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        } else if(Side() == RIGHT && Team() == RED) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(11.5,-13.5,Math.toRadians(180)))
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        } else if(Side() == LEFT && Team() == BLUE) {
//                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(11.5, 13.5, Math.toRadians(0)))
//                                    .build();
//                            drive.followTrajectory(CyclePark3);
//                        }
//                    }
//                    if (isStopRequested()) return;
//                    break;
//            }
            closeLatch();
        }
        PoseStorage.team = Team();
        PoseStorage.currentPose = drive.getPoseEstimate();
        sleep(3000);
    }
}