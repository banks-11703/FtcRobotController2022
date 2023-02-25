package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config

@Autonomous
public class AutoRemote extends AutoCommonRemote {

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initialization();

        waitForStart();
        timeStampStart = runtime.time();
        if(scoreFar) {
            if (Side() == LEFT) {
                turntableMod = -1;
            } else {
                turntableMod = 1;
            }
        } else {
            if (Side() == LEFT) {
                turntableMod = 1;
            } else {
                turntableMod = -1;
            }
        }

        starting();
        closeClaw();
         if (Mode() == 1) {//just driving
             drive.setPoseEstimate(PoseStorage.currentPose);
             telemetry.addData("Start Pos",StartingPos());
             telemetry.addData("Robot Location",drive.getPoseEstimate());
             telemetry.update();
             moveLift(1550);

             //strafe left
             Trajectory Drive1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(12,-60,Math.toRadians(90)))
                    .build();
             drive.followTrajectory(Drive1);
             //turn to junction while driving to the junction
             turnTable(-825);
             Trajectory Drive2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(12,-28,Math.toRadians(90)))
                    .build();
             drive.followTrajectory(Drive2);
             //score1
             openClaw();
             sleep(400);
             turnTable(0);
             //spin to face cones
             Trajectory Drive3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(9,-13.5,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
             drive.followTrajectory(Drive3);
             //intake1
             Trajectory Drive4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                     .lineToLinearHeading(new Pose2d(65,-14,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                     .build();
             drive.followTrajectory(Drive4);
             //grab cone and lift
             moveLift(250);
             sleep(1400);
             closeClaw();
             sleep(200);
             moveLift(1550);
             //score2
             Trajectory Drive5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                     .lineToLinearHeading(new Pose2d(3,-13.5,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                     .build();
             drive.followTrajectory(Drive5);
             //score cone
             turnTable(-825);
             sleep(500);
             openClaw();
             sleep(300);
             turnTable(0);
             //intake2
             Trajectory Drive7 = drive.trajectoryBuilder(drive.getPoseEstimate())
                     .lineToLinearHeading(new Pose2d(65,-16,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                     .build();
             drive.followTrajectory(Drive7);
             //grab cone
             sleep(750);
             moveLift(150);
             sleep(1400);
             closeClaw();
             sleep(200);
             moveLift(1550);
             //score3
             Trajectory Drive8 = drive.trajectoryBuilder(drive.getPoseEstimate())
                     .lineToLinearHeading(new Pose2d(2.75,-14,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                     .build();
             drive.followTrajectory(Drive8);
             //score
             turnTable(-825);
             sleep(600);
             openClaw();
             sleep(400);
             turnTable(0);
             //park
             switch (autoParkPosition) {
                 case 1:
                     Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                             .lineToLinearHeading(new Pose2d(36.5,-13.5,Math.toRadians(180)))
                             .build();
                     drive.followTrajectory(CyclePark2);
                     if (isStopRequested()) return;
                     break;
                 case 2:
                     if (Side() == 0) {
                         Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                 .lineToLinearHeading(new Pose2d(11.5,-13.5,Math.toRadians(180)))
                                 .build();
                         drive.followTrajectory(CyclePark3);
                     } else {
                         Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                 .lineToLinearHeading(new Pose2d(59.5,-13.5,Math.toRadians(180)))
                                 .build();
                         drive.followTrajectory(CyclePark1);
                     }
                     if (isStopRequested()) return;
                     break;
                 default:
                     if (Side() == 0) {
                         Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                 .lineToLinearHeading(new Pose2d(59.5,-13.5,Math.toRadians(180)))
                                 .build();
                         drive.followTrajectory(CyclePark1);
                     } else {
                         Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                 .lineToLinearHeading(new Pose2d(11.5,-16.5,Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                 .build();
                         drive.followTrajectory(CyclePark3);
                     }
                     if (isStopRequested()) return;
                     break;
             }
             moveLift(0);
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
            moveToScore();

            while (opModeIsActive() && !isStopRequested() && !armDone && TimeSinceStart() <= 27) {
                doLiftTasks();
                doShootoutTasks();
            }
            closeClaw();
            moveShootout(0);
            openShooterClaw();
            liftShooterClaw(false,0);
            openClaw();

//            Park
            drive.setPoseEstimate(PoseStorage.currentPose);
            switch (autoParkPosition) {
                case 1:
                    if (Side() == RIGHT && Team() == BLUE) {
                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(park2)
                                .build();
                        drive.followTrajectory(CyclePark2);
                    } else if(Side() == LEFT && Team() == RED) {
                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-36.5,-13.5,Math.toRadians(0)))
                                .build();
                        drive.followTrajectory(CyclePark2);
                    } else if(Side() == RIGHT && Team() == RED) {
                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(36.5,-13.5,Math.toRadians(180)))
                                .build();
                        drive.followTrajectory(CyclePark2);
                    } else if(Side() == LEFT && Team() == BLUE) {
                        Trajectory CyclePark2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(36.5,13.5,Math.toRadians(0)))
                                .build();
                        drive.followTrajectory(CyclePark2);
                    }
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (Side() == 0) {
                        if (Side() == RIGHT && Team() == BLUE) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(park3)
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        } else if(Side() == LEFT && Team() == RED) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-11.5,-13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        } else if(Side() == RIGHT && Team() == RED) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(11.5,-13.5,Math.toRadians(180)))
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        } else if(Side() == LEFT && Team() == BLUE) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(11.5,13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        }
                    } else {
                        if (Side() == RIGHT && Team() == BLUE) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(park1)
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        } else if(Side() == LEFT && Team() == RED) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-59.5,-13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        } else if(Side() == RIGHT && Team() == RED) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(59.5,-13.5,Math.toRadians(180)))
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        } else if(Side() == LEFT && Team() == BLUE) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(59.5,13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        }
                    }

                    if (isStopRequested()) return;
                    break;
                default:
                    if (Side() == 0) {
                        if (Side() == RIGHT && Team() == BLUE) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(park1)
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        } else if(Side() == LEFT && Team() == RED) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-59.5,-13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        } else if(Side() == RIGHT && Team() == RED) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(59.5,-13.5,Math.toRadians(180)))
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        } else if(Side() == LEFT && Team() == BLUE) {
                            Trajectory CyclePark1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(59.5,13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark1);
                        }
                    } else {
                        if (Side() == RIGHT && Team() == BLUE) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(park3)
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        } else if(Side() == LEFT && Team() == RED) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-11.5,-13.5,Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        } else if(Side() == RIGHT && Team() == RED) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(11.5,-13.5,Math.toRadians(180)))
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        } else if(Side() == LEFT && Team() == BLUE) {
                            Trajectory CyclePark3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(11.5, 13.5, Math.toRadians(0)))
                                    .build();
                            drive.followTrajectory(CyclePark3);
                        }
                    }
                    if (isStopRequested()) return;
                    break;
            }
            closeLatch();
        }
        PoseStorage.team = Team();
        PoseStorage.currentPose = drive.getPoseEstimate();
        sleep(3000);
    }
}