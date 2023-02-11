package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "DriveCodeCommonNoAutoHome", group = "Linear Opmode")
@Config
@Disabled
public class DriveCodeCommonNotBryce extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    double maxvel = 2787.625;
    double Timestamp = 0;
    double latchTimeStamp = 0;
    double latchTimeStamp2 = 0;
    double latchTimeStamp3 = 0;
    double TimestampSclaw = 0;
    boolean autoHome = false;
    boolean autoCounterClockwise = false;
    boolean autoClockwise = false;
    boolean atHome = false;
    boolean button_dpaddown2_was_pressed = false;
    boolean button_dpadup2_was_pressed = false;
    boolean button_dpadup1_was_pressed = false;
    boolean button_dpaddown1_was_pressed = false;
    boolean button_dpadright1_was_pressed = false;
    boolean button_bumperleft2_was_pressed = false;
    boolean button_bumperright2_was_pressed = false;
    boolean button_dpadleft1_was_pressed = false;
    boolean button_dpadleft2_was_pressed = false;
    boolean button_dpadright2_was_pressed = false;
    boolean button_x2_was_pressed = false;
    boolean button_a2_was_pressed = false;
    boolean button_y2_was_pressed = false;
    boolean button_b2_was_pressed = false;
    boolean resettingEncoders = false;
    boolean shootout = false;
    boolean readytoclose;
    boolean readytocloseSCLAW;
    boolean clawrunonce = false;
    boolean turntoleft = false;
    boolean lastwasleft = false;
    boolean turningtoleft = false;
    boolean lastwasforward = false;
    boolean turntoright = false;
    boolean lastwasright = true;
    boolean turningtoright = false;
    boolean shooterEngaged;
    boolean button_back2_was_pressed = false;
    boolean button_a_was_pressed;
    boolean button_x_was_pressed;
    boolean firstrun = true;
    boolean coneinhand;
    double ttrue;
    boolean resettingAutoHome = false;
    int liftLevel = 1;
    int hclaw = 0;
    int lclaw = 0;
    int latch;
    double turntimer = 0;
    int yMod = 0;
    int xMod = 0;
    double autoHomeSpeed;
    double sliftheight = 0;
    int ttpos = 0;
    int liftPreciseLocation;
    double liftPrecisePower;
    int conelevel;
    int team;
    boolean ttInDangerZone;
    double[] coneHeights = {0.00, 0.10, 0.22, 0.3225, 0.47};
    double[] coneHeightsClear = {0.00, 0.55, 0.72, 0.91, 1.0};

    boolean readyToAutoClose = true;
    boolean tryingToResetShooterEncoder = false;
    final int CLOSED = 0;
    final int OPENING = 1;
    final int OPENED = 2;
    int latchState = CLOSED;

    boolean movingLClaw = false;

    public double liftLevel() {
        return liftLevel % 5;
    }

    public double hClawToggle() {
        return hclaw % 2;
    }

    public double lClawToggle() {
        return lclaw % 2;
    }

    public double LatchToggle() {
        return latch % 2;
    }

    public double TimeSinceStamp() {
        return runtime.time() - Timestamp;
    }

    public double latchTimeSinceStamp() {
        return runtime.time() - latchTimeStamp;
    }

    public double latchTimeSinceStamp2() {
        return runtime.time() - latchTimeStamp2;
    }

    public double latchTimeSinceStamp3() {
        return runtime.time() - latchTimeStamp3;
    }

    public double sclawTimeSinceStamp() {
        return runtime.time() - TimestampSclaw;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void Initialization() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.slift.scaleRange(0.01, 0.32);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.turntable.setTargetPosition(drive.turntable.getCurrentPosition());
        drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        drive.mainLift.setTargetPosition(drive.mainLift.getCurrentPosition());
    }

    public void OdoDriving() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -((gamepad1.right_stick_x) / 2)
                )
        );
    }

    public void RawDriving() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.leftFront.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.5));
        drive.rightFront.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.5));
        drive.rightRear.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.5));
        drive.leftRear.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.5));

    }

    public void Toggles() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (gamepad2.back && !button_back2_was_pressed) {
            resettingAutoHome = true;
            button_back2_was_pressed = true;
        } else if (!gamepad2.back && button_back2_was_pressed) {
            button_back2_was_pressed = false;
        }
        if (gamepad2.dpad_down && !button_dpaddown2_was_pressed) {

            button_dpaddown2_was_pressed = true;
        } else if (!gamepad2.dpad_down && button_dpaddown2_was_pressed) {
            button_dpaddown2_was_pressed = false;
        }
        if (gamepad2.dpad_up && !button_dpadup2_was_pressed) {

            button_dpadup2_was_pressed = true;
        } else if (!gamepad2.dpad_up && button_dpadup2_was_pressed) {
            button_dpadup2_was_pressed = false;
        }
        if (gamepad1.dpad_down && !button_dpaddown1_was_pressed) {
            button_dpaddown1_was_pressed = true;
        } else if (!gamepad1.dpad_down && button_dpaddown1_was_pressed) {
            button_dpaddown1_was_pressed = false;
        }
        if (gamepad1.dpad_up && !button_dpadup1_was_pressed) {
            button_dpadup1_was_pressed = true;
        } else if (!gamepad1.dpad_up && button_dpadup1_was_pressed) {
            button_dpadup1_was_pressed = false;
        }
        if (gamepad1.dpad_right && !button_dpadright1_was_pressed) {
            button_dpadright1_was_pressed = true;
        } else if (!gamepad1.dpad_right && button_dpadright1_was_pressed) {
            button_dpadright1_was_pressed = false;
        }
        if (gamepad2.left_bumper && !button_bumperleft2_was_pressed) {
            button_bumperleft2_was_pressed = true;
        } else if (!gamepad1.left_bumper && button_bumperleft2_was_pressed) {
            button_bumperleft2_was_pressed = false;
        }
        if (gamepad2.right_bumper && !button_bumperright2_was_pressed) {
            button_bumperright2_was_pressed = true;
        } else if (!gamepad1.right_bumper && button_bumperright2_was_pressed) {
            button_bumperright2_was_pressed = false;
        }
        if (gamepad2.dpad_left && !button_dpadleft2_was_pressed) {
            button_dpadleft2_was_pressed = true;
            autoClockwise = true;
            autoHome = false;
            autoCounterClockwise = false;
        } else if (!gamepad2.dpad_left && button_dpadleft2_was_pressed) {
            button_dpadleft2_was_pressed = false;
        }
        if (gamepad2.dpad_right && !button_dpadright2_was_pressed) {
            button_dpadright2_was_pressed = true;
            autoCounterClockwise = true;
            autoHome = false;
            autoClockwise = false;
        } else if (!gamepad2.dpad_right && button_dpadright2_was_pressed) {
            button_dpadright2_was_pressed = false;
        }
        if (gamepad1.dpad_left && !button_dpadleft1_was_pressed) {
            button_dpadleft1_was_pressed = true;
        } else if (!gamepad1.dpad_left && button_dpadleft1_was_pressed) {
            button_dpadleft1_was_pressed = false;
        }
        if (gamepad1.dpad_left && !button_dpadleft1_was_pressed) {
            button_dpadleft1_was_pressed = true;
        } else if (!gamepad1.dpad_left && button_dpadleft1_was_pressed) {
            button_dpadleft1_was_pressed = false;
        }
        if (gamepad2.x && !button_x2_was_pressed) {
            hclaw++;
            button_x2_was_pressed = true;
        } else if (!gamepad2.x && button_x2_was_pressed) {
            button_x2_was_pressed = false;
        }
        if (gamepad1.x && !button_x_was_pressed) {
            lclaw++;
            button_x_was_pressed = true;
        } else if (!gamepad1.x && button_x_was_pressed) {
            button_x_was_pressed = false;
        }
        if (gamepad2.a && !button_a2_was_pressed && liftLevel > 1) {
//            if(firstrun){firstrun = false;}
            liftLevel--;
            button_a2_was_pressed = true;
        } else if (!gamepad2.a && button_a2_was_pressed) {
            button_a2_was_pressed = false;
        }
        if (gamepad1.a && !button_a_was_pressed) {
            latch++;
            button_a_was_pressed = true;
        } else if (!gamepad1.a && button_a_was_pressed) {
            button_a_was_pressed = false;
        }
        if (gamepad2.y && !button_y2_was_pressed) {
            autoHome = true;
            autoClockwise = false;
            autoCounterClockwise = false;
            button_y2_was_pressed = true;
        } else if (!gamepad2.y && button_y2_was_pressed) {
            button_y2_was_pressed = false;
        }
        if (gamepad2.b && !button_b2_was_pressed && liftLevel < 4) {
//            if(firstrun){firstrun = false;}
            liftLevel++;
            button_b2_was_pressed = true;
        } else if (!gamepad2.b && button_b2_was_pressed) {
            button_b2_was_pressed = false;
        }
        if (gamepad2.left_stick_button && gamepad2.right_stick_button && gamepad1.start && !resettingEncoders) {
            resettingEncoders = true;
        } else if (!(gamepad2.left_stick_button && gamepad2.right_stick_button && gamepad1.start) && resettingEncoders) {
            drive.mainLift.setPower(0);
            resettingEncoders = false;
        }
    }

    public void Lift() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if (liftLevel() == 1) { // intake
            if (coneinhand) {//whats up with all the if else statements that do the same thing? -Owen
                if (conelevel == 4) {
                    liftPreciseLocation = 10;
                } else if (conelevel == 3) {
                    liftPreciseLocation = 10;
                } else if (conelevel == 2) {
                    liftPreciseLocation = 10;
                } else if (conelevel == 1) {
                    liftPreciseLocation = 10;
                } else {
                    liftPreciseLocation = 10;
                }
            } else {
                if (conelevel == 4) {
                    liftPreciseLocation = 10;
                } else if (conelevel == 3) {
                    liftPreciseLocation = 10;
                } else if (conelevel == 2) {
                    liftPreciseLocation = 10;
                } else if (conelevel == 1) {
                    liftPreciseLocation = 10;
                } else {
                    liftPreciseLocation = 10;
                }
            }

            liftPrecisePower = 1;
        } else if (liftLevel() == 2) { // low 1050
            liftPreciseLocation = 625;
            liftPrecisePower = 1;
        } else if (liftLevel() == 3) { // mid 1800
            liftPreciseLocation = 1075;
            liftPrecisePower = 1;
        } else if (liftLevel() == 4) { // high
            liftPreciseLocation = 1525;
            liftPrecisePower = 1;
        }

        if(autoHome) {
            liftLevel = 1;
            if(drive.mainLift.getCurrentPosition() >= 750 || Math.abs(drive.turntable.getCurrentPosition()) <= 25) {
                drive.mainLift.setTargetPosition(liftPreciseLocation);
                drive.mainLift.setPower(1);
            } else {
                drive.mainLift.setTargetPosition(liftPreciseLocation);
                drive.mainLift.setPower(0);
            }
        } else {
            if(ttInDangerZone){//Im not optimizing this -Owen (lifts lift above danger zone)
                drive.mainLift.setTargetPosition(925);
            }else if (gamepad2.dpad_down) {
                drive.mainLift.setTargetPosition(liftPreciseLocation - 100);
            } else {
                drive.mainLift.setTargetPosition(liftPreciseLocation);
            }
            drive.mainLift.setPower(liftPrecisePower);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    public void Claw() {
        if (lClawToggle() == 0) {
            SClaw(true);
        } else {
            SClaw(false);
        }
        if (hClawToggle() == 0) {
            MClaw(true);
            if (!clawrunonce) {
                Timestamp = runtime.time();
                clawrunonce = true;
            }
            if (TimeSinceStamp() >= 1000) {
                readytoclose = true;
            }
            telemetry.addData("Time Since Opened", TimeSinceStamp());

            telemetry.addData("Claw:", "Open");
        } else if (hClawToggle() == 1) {
            telemetry.addData("Claw:", "Closed");
            readytoclose = false;
            if (!readytocloseSCLAW) {
                TimestampSclaw = runtime.time();
                readytocloseSCLAW = true;
            }
            if(liftLevel() == 1) {
                if(lClawToggle() == 1) {
                    lclaw++;
                }
            }
            MClaw(false);
        }
    }

    public void TurnTable() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if(liftLevel() == 2 && !(Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.05) && (autoHome || autoClockwise || autoCounterClockwise)){//if in danger zone for cone
            ttInDangerZone = true;
        }else{//normal manual controls
            ttInDangerZone = false;
        }

        if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.05) {//manual turntable control
            autoHome = false;
            autoCounterClockwise = false;
            autoClockwise = false;
            drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.turntable.setPower((gamepad2.right_trigger - gamepad2.left_trigger));
        } else if (autoClockwise && !autoHome && !autoCounterClockwise && liftLevel > 1) {//snap left
            drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ttpos = -825;
            drive.turntable.setTargetPosition(ttpos);
            drive.turntable.setPower(1);
            autoHome = false;
            autoCounterClockwise = false;
            if(liftLevel() == 2 && drive.mainLift.getCurrentPosition() < 900){
                drive.turntable.setPower(0);
            }
            if(Math.abs(drive.turntable.getCurrentPosition() - ttpos) < 10 && liftLevel() == 2){
                autoClockwise = false;
            }
        } else if (autoCounterClockwise && !autoHome && !autoClockwise && liftLevel > 1) {//snap right
            drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ttpos = 825;
            drive.turntable.setTargetPosition(ttpos);
            drive.turntable.setPower(1);
            autoHome = false;
            autoClockwise = false;
            if(liftLevel() == 2 && drive.mainLift.getCurrentPosition() < 900){
                drive.turntable.setPower(0);
            }
            if(Math.abs(drive.turntable.getCurrentPosition() - ttpos) < 10 && liftLevel() == 2){
                autoCounterClockwise = false;
            }
        } else if (autoHome && !autoCounterClockwise && !autoClockwise) {//automatically centering on intake
            drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ttpos = 0;
            drive.turntable.setTargetPosition(ttpos);
            drive.turntable.setPower(1);
            autoCounterClockwise = false;
            autoClockwise = false;
            if(liftLevel() == 2 && drive.mainLift.getCurrentPosition() < 900){
                drive.turntable.setPower(0);
            }
            if(Math.abs(drive.turntable.getCurrentPosition() - ttpos) < 10 && liftLevel() == 2){
                autoHome = false;
            }
            if(drive.mainLift.getCurrentPosition()<=175) {
                autoHome = false;
            }
        } else {//turntable not moving
            drive.turntable.setPower(0);
        }

        if(resettingAutoHome){
            drive.turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resettingAutoHome = false;
        }




//            drive.turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            double tablePower;
//            tablePower = gamepad2.right_trigger - gamepad2.left_trigger;
//            telemetry.addData("Turn Table Power", tablePower);
//            drive.turntable.setPower(tablePower);

    }

    public void Telemetry() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Update everything. Odometry. Etc.
        drive.update();
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        // Print pose to telemetry
        telemetry.addData("Trying to go home", autoHome);
        telemetry.addData("Lift", drive.mainLift.getCurrentPosition());
        telemetry.addData("LiftLevel", liftLevel());
        telemetry.addData("Turntable Position", drive.turntable.getCurrentPosition());
        telemetry.addData("TT Offset", motorOffset(drive.turntable));
        telemetry.addData("s claw timer", sclawTimeSinceStamp());
        telemetry.addData("slift", drive.slift.getPosition());
        telemetry.addData("Shooter Ticks", drive.shooter.getCurrentPosition());
        telemetry.update();
    }

    public void Lights() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("clawPos",drive.claw.getPosition());
        drive.intakeRedLED.setMode(DigitalChannel.Mode.OUTPUT);
        drive.intakeGreenLED.setMode(DigitalChannel.Mode.OUTPUT);

        if(drive.claw.getPosition() > 0.73){//if open
            drive.intakeRedLED.setState(false);
            drive.intakeGreenLED.setState(true);
        }else{//if closed
            drive.intakeRedLED.setState(true);
            drive.intakeGreenLED.setState(false);
        }





        //check if you can just set cp1 and cp2 in code
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        if (!isStarted()) {
//            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        } else if (PoseStorage.team == 0) {
//            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        } else {
//            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        }
    }

    public void ShootOut() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        if((gamepad1.left_bumper || gamepad1.right_bumper)) {
            switch(latchState) {
                case CLOSED:
                    Latch(true);
                    latchTimeStamp = runtime.time(TimeUnit.SECONDS);
                    latchState = OPENING;
                case OPENING:
                    if(latchTimeSinceStamp() >= 0.6) {
                        latchState = OPENED;
                    }
                case OPENED:
                    if (gamepad1.left_bumper) {
                        drive.shooter.setPower(-1);
                    } else if (gamepad1.right_bumper) {
                        drive.shooter.setPower(1);
                    }
            }
        } else if(button_dpadleft1_was_pressed) {
            Latch(false);
            latchState = CLOSED;
            drive.shooter.setPower(0);
        } else if(button_dpadright1_was_pressed) {
            Latch(true);
            latchState = OPENED;
            drive.shooter.setPower(0);
        } else if(latchState == OPENED && drive.shooter.getCurrentPosition() <= 10 && readyToAutoClose) {
            Latch(false);
            latchTimeStamp3 = runtime.time(TimeUnit.SECONDS);
            readyToAutoClose = false;
            drive.shooter.setPower(0);
        } else if(latchTimeSinceStamp3() >= 0.6 && !readyToAutoClose) {
            if(!drive.turnlimiter.getState()) {
                latchState = CLOSED;
            } else {
                Latch(true);
            }
            drive.shooter.setPower(0);
        } else if(latchState == OPENED && drive.shooter.getCurrentPosition() >= 200) {
            readyToAutoClose = true;
            drive.shooter.setPower(0);
        }

        if(!tryingToResetShooterEncoder) {
            if(!drive.turnlimiter.getState()) {
                latchTimeStamp2 = runtime.time(TimeUnit.SECONDS);
                tryingToResetShooterEncoder = true;
            }
        } else {
            if(latchTimeSinceStamp2() >= 1 && latchState == CLOSED) {
                drive.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                tryingToResetShooterEncoder = false;
            }
        }

        if(button_dpaddown1_was_pressed) {
            if(!movingLClaw) {
                if(lClawToggle() == 0) {
                    lclaw++;
                }
            }
            movingLClaw = true;
            double currentPos = drive.slift.getPosition();
            drive.slift.setPosition(currentPos-0.1);
        } else if(button_dpadup1_was_pressed) {
            if(!movingLClaw) {
                if(lClawToggle() == 0) {
                    lclaw++;
                }
            }
            movingLClaw = true;
            double currentPos = drive.slift.getPosition();
            drive.slift.setPosition(currentPos+0.1);
        } else {
            movingLClaw = false;
        }

//        if ((gamepad1.a || !drive.turnlimiter.getState()) && !gamepad1.left_bumper && !gamepad1.right_bumper) {
//            timestamponce = false;
//            drive.shooter.setPower(0);
//            Latch(false);
//            telemetry.addData("slift", drive.slift.getPosition());
//            telemetry.addData("Shooter Ticks", drive.shooter.getCurrentPosition());
//
//        } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
//            Latch(true);
//            if (!timestamponce) {
//                latchTimestamp = runtime.time(TimeUnit.SECONDS);
//                timestamponce = true;
//            }
//            if (latchTimeSinceStamp() >= 0.6) {
//
//            }
//        } else {
//            drive.shooter.setPower(0);
//        }
//        if (!coneinhand) {
//            drive.slift.setPosition(coneHeights[conelevel]);
//        } else {
//            drive.slift.setPosition(coneHeightsClear[conelevel]);
//        }
    }

    public void MClaw(boolean open) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open) {
            drive.claw.setPosition(0.76);//0.225
        } else {
            if (sclawTimeSinceStamp() >= 1 && readytocloseSCLAW) {
//                lclaw = 0;
//                readytocloseSCLAW = false;
            }
            drive.claw.setPosition(0.66);//0.95
        }
    }

    public void SClaw(boolean open) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open) {
            drive.sclaw.setPosition(0.05);//0.225
        } else {
            drive.sclaw.setPosition(0.29);//0.95
        }
    }

    public void Latch(boolean open) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open) {
            drive.latch.setPosition(0);//0.225
        } else {
            drive.latch.setPosition(1);//0.95
        }
    }

    public void ColorSensor() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("red", drive.csensor.red());
        telemetry.addData("blue", drive.csensor.blue());
        telemetry.addData("distance", drive.csensor.getDistance(DistanceUnit.INCH));
//        if (readytoclose) {
//            if (PoseStorage.team == 0) {
//                if (drive.csensor.blue() <= 200 && drive.csensor.red() >= 200 && drive.csensor.getDistance(DistanceUnit.INCH) <= 0.25 && conelevel == 0) {
//                    lclaw = 0;
//                    coneinhand = true;
//                }
//            } else {
//                if (drive.csensor.blue() >= 200 && drive.csensor.red() <= 200 && drive.csensor.getDistance(DistanceUnit.INCH) <= 0.25 && conelevel == 0) {
//                    lclaw = 0;
//                    coneinhand = true;
//                }
//            }
//        }
    }

    public int motorOffset(DcMotorEx motor) {
        return Math.abs((motor.getTargetPosition()) - (motor.getCurrentPosition()));

    }
}