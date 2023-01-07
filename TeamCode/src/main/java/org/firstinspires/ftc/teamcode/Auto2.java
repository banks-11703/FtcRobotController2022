package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

        } else if (Mode() == 1) {//cycling
            moveToScore();
        } else if (Mode() == 2) {//Just parking
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
        } else if(Mode() == 3) {//shootout
            moveToScore();

            while(opModeIsActive() && !isStopRequested() && !armDone && TimeSinceStart() <= 27000){
                doLiftTasks();
                doShootoutTasks();
            }

            //Park
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

        PoseStorage.team = team % 2;
        PoseStorage.currentPose = drive.getPoseEstimate();
        sleep(3000);
    }
}