package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config

@Autonomous
public class Auto2 extends AutoCommon {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initialization();

        waitForStart();

        starting();

        if (Mode() == 1) {//doing nothing

        } else if (Mode() == 1) {//cycling
            movetoscore();
        } else if (Mode() == 2) {//Just parking
            movetoscore();

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
        } else if(Mode() == 3) {//shootout. Do this next

        }
    }
}