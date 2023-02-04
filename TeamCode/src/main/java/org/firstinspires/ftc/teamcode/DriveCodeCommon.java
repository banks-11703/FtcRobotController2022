package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "DriveCodeCommon", group = "Linear Opmode")
@Config
@Disabled
public class DriveCodeCommon extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    double maxvel = 2787.625;
    double Timestamp = 0;
    double latchTimestamp = 0;
    double TimestampSclaw = 0;
    boolean autoHome = false;
    boolean atHome = false;
    boolean button_dpaddown2_was_pressed = false;
    boolean button_dpadup2_was_pressed = false;
    boolean button_dpadup1_was_pressed = false;
    boolean button_dpaddown1_was_pressed = false;
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
    boolean button_a_was_pressed;
    boolean button_x_was_pressed;
    boolean firstrun = true;
    boolean timestamponce;
    boolean coneinhand;
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
    double[] coneHeights = {0.00, 0.10, 0.22, 0.3225, 0.47};
    double[] coneHeightsClear = {0.00, 0.55, 0.72, 0.91, 1.0};

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
        return runtime.time() - latchTimestamp;
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
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.slift.scaleRange(0.01, 0.32);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // this is what we get from autonomous
        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.turntable.setTargetPosition(drive.turntable.getCurrentPosition());
        drive.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        if (gamepad1.dpad_down && !button_dpaddown1_was_pressed && conelevel > 0) {
            conelevel--;
            button_dpaddown1_was_pressed = true;
        } else if (!gamepad1.dpad_down && button_dpaddown1_was_pressed) {
            button_dpaddown1_was_pressed = false;
        }
        if (gamepad1.dpad_up && !button_dpadup1_was_pressed && conelevel < 4) {
            conelevel++;
            button_dpadup1_was_pressed = true;
        } else if (!gamepad1.dpad_up && button_dpadup1_was_pressed) {
            button_dpadup1_was_pressed = false;
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
            if (coneinhand) {
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

            liftPrecisePower = 0.5;
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

        if (gamepad2.dpad_down) {
            drive.mainLift.setTargetPosition(liftPreciseLocation - 100);
        } else {
            drive.mainLift.setTargetPosition(liftPreciseLocation);
        }
        drive.mainLift.setPower(liftPrecisePower);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            MClaw(false);
        }
    }

    public void TurnTable() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (autoHome) {
            ttpos = 0;
            drive.turntable.setPower(0.6);
            if (!drive.turntable.isBusy()) {
                atHome = true;
                autoHome = false;
            }
        } else if (gamepad2.left_bumper && liftLevel > 2) {
            atHome = false;
            ttpos = -825;
        } else if (gamepad2.right_bumper && liftLevel > 2) {
            atHome = false;
            ttpos = 825;
        } else {
            ttpos += Math.round(75 * (gamepad2.right_trigger - gamepad2.left_trigger));
        }
        drive.turntable.setTargetPosition(ttpos);
        drive.turntable.setPower(0.75);
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
        telemetry.update();
    }

    public void Lights() {
        //check if you can just set cp1 and cp2 in code
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (!isStarted()) {
            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (PoseStorage.team == 0) {
            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            drive.lightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }

    public void ShootOut() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if ((gamepad1.a || !drive.turnlimiter.getState()) && !gamepad1.left_bumper && !gamepad1.right_bumper) {
            timestamponce = false;
            drive.shooter.setPower(0);
            drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Latch(false);
            telemetry.addData("slift", drive.slift.getPosition());
            telemetry.addData("Shooter Ticks", drive.shooter.getCurrentPosition());

        } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
            Latch(true);
            if (!timestamponce) {
                latchTimestamp = runtime.time(TimeUnit.SECONDS);
                timestamponce = true;
            }
            if (latchTimeSinceStamp() >= 0.6) {
                if (gamepad1.left_bumper) {
                    drive.shooter.setPower(-1);
                } else if (gamepad1.right_bumper) {
                    drive.shooter.setPower(1);
                } else {
                    drive.shooter.setPower(0);
                }
            }
        } else {
            drive.shooter.setPower(0);
        }
        if (!coneinhand) {
            drive.slift.setPosition(coneHeights[conelevel]);
        } else {
            drive.slift.setPosition(coneHeightsClear[conelevel]);
        }
    }

    public void MClaw(boolean open) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if (open) {
            drive.claw.setPosition(0.76);//0.225
        } else {
            if (sclawTimeSinceStamp() >= 1 && readytocloseSCLAW) {
                lclaw = 0;
                readytocloseSCLAW = false;
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