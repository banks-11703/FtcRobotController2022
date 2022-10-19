package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class DriveCode extends LinearOpMode {
    boolean autoHome;
    boolean turntoforward = false;
    boolean turntoright = false;
    boolean turntoleft = false;
    boolean lastwasforward = false;
    boolean lastwasright = false;
    boolean lastwasleft = false;
    boolean turningtoleft = false;
    boolean turningtoright = false;
    boolean button_dpaddown2_was_pressed = false;
    boolean button_dpadup2_was_pressed = false;
    boolean button_dpadleft2_was_pressed = false;
    boolean button_dpadright2_was_pressed = false;
    boolean button_x2_was_pressed = false;
    double level;
    double claw;
    int yMod = 0;
    int xMod = 0;

    public double Level() {
        return level % 5;
    }

    public double Claw() {
        return claw % 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up) {
                yMod = 1;
            } else if (gamepad1.dpad_down) {
                yMod = -1;
            } else {
                yMod = 0;
            }
            if (gamepad1.dpad_right) {
                xMod = 1;
            } else if (gamepad1.dpad_left) {
                xMod = -1;
            } else {
                xMod = 0;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y + yMod,
                            -gamepad1.left_stick_x - xMod,
                            -gamepad1.right_stick_x
                    )
            );
            // && drive.lift.getCurrentPosition() >= -5600
            //&& drive.lift.getCurrentPosition() <= -100
//            if (Level() == 1) {
//                if (button_a_was_pressed) {
//                    drive.lift.setTargetPosition(200);
//                    drive.lift.setPower(0.5);
//                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    drive.lift.setPower(0);
//                } else {
//                    drive.lift.setPower(0);
//                }
//            } else if (Level() == 4) {
//
//                if (gamepad1.right_bumper) {
//                    drive.lift.setPower(-1);
//                } else if (gamepad1.left_bumper) {
//                    drive.lift.setPower(1);
//                } else {
//                    drive.lift.setPower(0);
//                }
//            }

            if (gamepad2.a) {
                drive.lift.setPower(-1);
            } else if (gamepad2.y) {
                drive.lift.setPower(1);
            } else {
                drive.lift.setPower(0);
            }
            if (Claw() == 0) {
                drive.claw.setPosition(0.3);//0.225
                telemetry.addData("Claw:", "Closed");
            } else if (Claw() == 1) {
                drive.claw.setPosition(0.150);//0.150
                telemetry.addData("Claw:", "Open");
            }
//        if (gamepad1.dpad_down && ((yPos - drive.arm.getPosition()) <= 0.02)) {
//            drive.arm.setPosition(yPos - 0.003);
//            yPos = drive.arm.getPosition();
//        } else if (gamepad1.dpad_up && (drive.arm.getPosition() - yPos) <= 0.02) {
//            drive.arm.setPosition(yPos + 0.003);
//            yPos = drive.arm.getPosition();
//        }

            if (turntoforward && !turningtoleft && !turningtoright) {
                autoHome = true;
            }
            if (turntoleft && !autoHome && !turningtoright) {
                if (!lastwasforward) {
                    autoHome = true;
                } else {
                    turningtoleft = true;
                }
            }
            if (turntoright && !autoHome && !turningtoleft) {
                if (!lastwasforward) {
                    autoHome = true;
                } else {
                    turningtoright = true;
                }
            }
            if (turningtoleft && lastwasforward) {
                telemetry.addData("TT at ", "Trying to turn Left");
                telemetry.update();
                drive.turntable.setTargetPosition(-825);
                drive.turntable.setPower(-0.4);
                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (drive.turntable.getTargetPosition() - drive.turntable.getCurrentPosition() > 10) {
                    drive.turntable.setPower(0);
                    turntoleft = false;
                    lastwasleft = true;
                    turningtoleft = false;
                    lastwasforward = false;
                    telemetry.addData("TT at ", "Left");
                    telemetry.update();
                }
            }
            if (turningtoright && lastwasforward) {
                telemetry.addData("TT at ", "Trying to turn right");
                drive.turntable.setTargetPosition(825);
                drive.turntable.setPower(0.4);
                drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (drive.turntable.getTargetPosition() - drive.turntable.getCurrentPosition() < 10) {
                    drive.turntable.setPower(0);
                    turntoright = false;
                    lastwasright = true;
                    lastwasforward = false;
                    turningtoright = false;
                    telemetry.addData("TT at ", "Right");
                    telemetry.update();
                }
            }
            if (autoHome && drive.turnlimiter.getState()) {
                telemetry.addData("TT is ", "Trying to go Home");
                telemetry.update();
                if (drive.turntable.getCurrentPosition() < 0) {
                    drive.turntable.setPower(0.3);
                } else {
                    drive.turntable.setPower(-0.3);
                }

            } else if (autoHome && !drive.turnlimiter.getState()) {
                drive.turntable.setPower(0);
                drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turntoforward = false;
                lastwasright = false;
                lastwasleft = false;
                autoHome = false;
                lastwasforward = true;
                telemetry.addData("TT at ", "Home");
                telemetry.update();
            } else {
                if (!turningtoright && !turningtoleft && !autoHome) {
                    drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    drive.turntable.setPower((gamepad2.right_trigger - gamepad2.left_trigger) / 2);
                }

            }
            // Update everything. Odometry. Etc.
            drive.update();
//            if (gamepad2.dpad_down && !button_dpaddown2_was_pressed) {
//                level++;
//                button_dpaddown2_was_pressed = true;
//            } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
//                button_dpaddown2_was_pressed = false;
//            }
            if (gamepad2.dpad_up && !button_dpadup2_was_pressed) {
                turntoforward = true;
                button_dpadup2_was_pressed = true;
            } else if (!gamepad2.dpad_up && button_dpadup2_was_pressed) {
                button_dpadup2_was_pressed = false;
            }
            if (gamepad2.dpad_left && !button_dpadleft2_was_pressed) {
                turntoleft = true;
                button_dpadleft2_was_pressed = true;
            } else if (!gamepad2.dpad_left && button_dpadleft2_was_pressed) {
                button_dpadleft2_was_pressed = false;
            }
            if (gamepad2.dpad_right && !button_dpadright2_was_pressed) {
                turntoright = true;
                button_dpadright2_was_pressed = true;
            } else if (!gamepad2.dpad_right && button_dpadright2_was_pressed) {
                button_dpadright2_was_pressed = false;
            }

            if (gamepad2.x && !button_x2_was_pressed) {
                claw++;
                button_x2_was_pressed = true;
            } else if (!gamepad2.x && button_x2_was_pressed) {
                button_x2_was_pressed = false;
            }
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Print pose to telemetry
            telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
            telemetry.addData("TurnTable", drive.turntable.getCurrentPosition());
            telemetry.addData("Lift", drive.lift.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("TT is at home", lastwasforward);
            if (Level() == 1) {
                telemetry.addData("Level:", "Intake");
            } else if (Level() == 2) {
                telemetry.addData("Level:", "Low");
            } else if (Level() == 3) {
                telemetry.addData("Level:", "Mid (like you)");
            } else if (Level() == 4) {
                telemetry.addData("Level:", "High (as a kite)");
            } else {
                telemetry.addData("Manual", "Control");
            }
            telemetry.update();
        }
    }
}

