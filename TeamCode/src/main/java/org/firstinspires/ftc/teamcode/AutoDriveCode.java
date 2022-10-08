package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java and TrajectorySequenceRunnerCancelable.java
 * classes. Please ensure that these files are copied into your own project.
 */
@TeleOp
public class AutoDriveCode extends LinearOpMode {

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        // Ensure that the contents are copied over from https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/SampleMecanumDriveCancelable.java
        // and https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TrajectorySequenceRunnerCancelable.java
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        int sign = 1;
        int isRed = 0;
        int aButtonStatus = 0;
        Pose2d startPos = new Pose2d(12,12,Math.toRadians(90));

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);


        while(!isStarted()){
            if(gamepad1.a && aButtonStatus == 0){
                aButtonStatus = 1;
                isRed = 1;
                sign = 1;
            }
            if(!gamepad1.a && aButtonStatus == 1){
                aButtonStatus = 2;
            }
            if(gamepad1.a && aButtonStatus == 2){
                aButtonStatus = 3;
                isRed = 2;
                sign = -1;
            }
            if(!gamepad1.a && aButtonStatus == 3){
                aButtonStatus = 4;
            }
            if(gamepad1.a && aButtonStatus == 4){
                aButtonStatus = 5;
                isRed = 0;
            }
            if(!gamepad1.a && aButtonStatus == 5){
                aButtonStatus = 0;
            }
            if(isRed == 0){
                telemetry.addData("State","Match Play");
            }
            if(isRed == 1){
                telemetry.addData("State","Testing Red");
            }
            if(isRed == 2){
                telemetry.addData("State","Testing Blue");
            }
            telemetry.update();
        }
        if(PoseStorage.team == 1 && isRed == 0) {
            sign = -1;
        }
        if (isRed == 0) {
            drive.setPoseEstimate(PoseStorage.currentPose);
        } else {
            drive.setPoseEstimate(startPos);
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();

            double currentGridX = drive.getPoseEstimate().getX() + (12 - (drive.getPoseEstimate().getX() % 24));
            double currentGridY = drive.getPoseEstimate().getY() + (12 - (drive.getPoseEstimate().getY() % 24));
            double currentGridHeading;
            if (Math.toDegrees(drive.getPoseEstimate().getHeading()) % 90 <= 45) {
                currentGridHeading = Math.toDegrees(drive.getPoseEstimate().getHeading()) - (Math.toDegrees(drive.getPoseEstimate().getHeading()) % 90);
            } else {
                currentGridHeading = Math.toDegrees(drive.getPoseEstimate().getHeading()) + (90 - (Math.toDegrees(drive.getPoseEstimate().getHeading()) % 90));
            }

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y*0.5,
                                    -gamepad1.left_stick_x*0.5,
                                    -gamepad1.right_stick_x*0.5
                            )
                    );

                    if (gamepad1.y) {
                        Pose2d gridAlign = new Pose2d(currentGridX,currentGridY,Math.toRadians(currentGridHeading));
                        Trajectory gridUp = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(gridAlign)
                                .build();
                        drive.followTrajectoryAsync(gridUp);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if(gamepad1.dpad_up){
                        Pose2d gridAlign = new Pose2d(currentGridX,currentGridY,Math.toRadians(currentGridHeading));
                        Pose2d gridMove = new Pose2d(currentGridX,currentGridY+(sign*24),Math.toRadians(currentGridHeading));
                        Trajectory gridUp = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(gridAlign)
                                .lineToLinearHeading(gridMove)
                                .build();
                        drive.followTrajectoryAsync(gridUp);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if(gamepad1.dpad_right){
                        Pose2d gridAlign = new Pose2d(currentGridX,currentGridY,Math.toRadians(currentGridHeading));
                        Pose2d gridMove = new Pose2d(currentGridX+(sign*24),currentGridY,Math.toRadians(currentGridHeading));
                        Trajectory gridRight = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(gridAlign)
                                .lineToLinearHeading(gridMove)
                                .build();
                        drive.followTrajectoryAsync(gridRight);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if(gamepad1.dpad_down){
                        Pose2d gridAlign = new Pose2d(currentGridX,currentGridY,Math.toRadians(currentGridHeading));
                        Pose2d gridMove = new Pose2d(currentGridX,currentGridY-(sign*24),Math.toRadians(currentGridHeading));
                        Trajectory gridDown = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(gridAlign)
                                .lineToLinearHeading(gridMove)
                                .build();
                        drive.followTrajectoryAsync(gridDown);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if(gamepad1.dpad_left){
                        Pose2d gridAlign = new Pose2d(currentGridX,currentGridY,Math.toRadians(currentGridHeading));
                        Pose2d gridMove = new Pose2d(currentGridX-(sign*24),currentGridY,Math.toRadians(currentGridHeading));
                        Trajectory gridLeft = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(gridAlign)
                                .lineToLinearHeading(gridMove)
                                .build();
                        drive.followTrajectoryAsync(gridLeft);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}