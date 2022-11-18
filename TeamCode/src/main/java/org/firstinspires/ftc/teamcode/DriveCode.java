package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class DriveCode extends LinearOpMode {
    boolean autoHome = false;
    boolean turnToForward = false;
    boolean turnToRight = false;
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
    boolean button_a2_was_pressed = false;
    boolean button_y2_was_pressed = false;
    boolean button_b2_was_pressed = false;
    boolean magnetwastouched;
    double level;
    double claw;
    double turntimer;
    int yMod = 0;
    int xMod = 0;
    int liftpos;
    int turnsensorcounter;

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
        drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (isStopRequested()) return;
        drive.mainLift.setTargetPosition(drive.mainLift.getCurrentPosition());
        turntimer = 0;
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
                            -((gamepad1.right_stick_x) / 2)
                    )
            );
            if (button_a2_was_pressed) { // intake
                drive.mainLift.setTargetPosition(0);
                drive.mainLift.setPower(0.5);
                drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (button_b2_was_pressed) { // low
                drive.mainLift.setTargetPosition(1150);
                drive.mainLift.setPower(1);
                drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (button_x2_was_pressed) { // mid
                drive.mainLift.setTargetPosition(2000);
                drive.mainLift.setPower(1);
                drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (button_y2_was_pressed) { // high
                drive.mainLift.setTargetPosition(2600);
                drive.mainLift.setPower(1);
                drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (Claw() == 0) {
                drive.claw.setPosition(0);//0.225
                telemetry.addData("Claw:", "Open");
            } else if (Claw() == 1) {
                drive.claw.setPosition(0.95);//0.150
                telemetry.addData("Claw:", "Closed");
            }


            if (autoHome && drive.turnlimiter.getState()) {
                telemetry.addData("TT is ", "Trying to go Home");
                if (turntimer > 0) {
                    drive.turntable.setPower(-0.5);
                } else {
                    drive.turntable.setPower(0.5);
                }
            } else if (autoHome && !drive.turnlimiter.getState() && turnsensorcounter == 0) {
                drive.turntable.setPower(0);
                turntimer = 0;
                turnToForward = false;
                lastwasright = false;
                lastwasleft = false;
                autoHome = false;
                lastwasforward = true;
                telemetry.addData("TT at ", "Home");
            } else {
                double tablePower = 0;
                if (!turningtoright && !turningtoleft && !autoHome) {
                    tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
                    turntimer += tablePower;
                }

                drive.turntable.setPower(tablePower / 1.5);
            }

            // Update everything. Odometry. Etc.
            drive.update();
            if (gamepad2.dpad_down && !button_dpaddown2_was_pressed) {
                claw++;
                button_dpaddown2_was_pressed = true;
            } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
                button_dpaddown2_was_pressed = false;
            }
            if (gamepad2.dpad_up && !button_dpadup2_was_pressed) {
                autoHome = true;
                button_dpadup2_was_pressed = true;
            } else if (!gamepad2.dpad_up && button_dpadup2_was_pressed) {
                button_dpadup2_was_pressed = false;
            }
//                if (gamepad2.dpad_left && !button_dpadleft2_was_pressed) {
//                    turntoleft = true;
//                    button_dpadleft2_was_pressed = true;
//                } else if (!gamepad2.dpad_left && button_dpadleft2_was_pressed) {
//                    button_dpadleft2_was_pressed = false;
//                }
//                if (gamepad2.dpad_right && !button_dpadright2_was_pressed) {
//                    turntoright = true;
//                    button_dpadright2_was_pressed = true;
//                } else if (!gamepad2.dpad_right && button_dpadright2_was_pressed) {
//                    button_dpadright2_was_pressed = false;
//                }

            if (gamepad2.x && !button_x2_was_pressed) {
                button_x2_was_pressed = true;
            } else if (!gamepad2.x && button_x2_was_pressed) {
                button_x2_was_pressed = false;
            }
            if (gamepad2.a && !button_a2_was_pressed) {
                button_a2_was_pressed = true;
            } else if (!gamepad2.a && button_a2_was_pressed) {
                button_a2_was_pressed = false;
            }
            if (gamepad2.y && !button_y2_was_pressed) {
                button_y2_was_pressed = true;
            } else if (!gamepad2.y && button_y2_was_pressed) {
                button_y2_was_pressed = false;
            }
            if (gamepad2.b && !button_b2_was_pressed) {
                button_b2_was_pressed = true;
            } else if (!gamepad2.y && button_b2_was_pressed) {
                button_b2_was_pressed = false;
            }
            if (!drive.turnlimiter.getState() && !magnetwastouched) {
                magnetwastouched = true;
                if (autoHome && turnsensorcounter != 0) {
                    turnsensorcounter--;
                } else if (!autoHome && turntimer >= 100) {
                    turnsensorcounter++;
                }
            } else if (drive.turnlimiter.getState() && magnetwastouched) {
                magnetwastouched = false;
            }
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Print pose to telemetry
            telemetry.addData("turn counter", turntimer);
            telemetry.addData("Turn Limiter", drive.turnlimiter.getState());
            telemetry.addData("Lift", drive.mainLift.getCurrentPosition());
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

