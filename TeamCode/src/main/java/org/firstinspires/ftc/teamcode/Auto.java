package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class Auto extends LinearOpMode {
    boolean button_b_was_pressed = false;
    boolean button_a_was_pressed = false;
    boolean button_x_was_pressed = false;
    int team = 0;// 0 = red 1 = blue    which team we are on
    int side = 0;// 0 = left 1 = right  are we left or right
    int Mode = 0;//0 = nothing          are we just parking or otherwise?
    int autoParkPosition = 0;
    int numOfTrajs = 1;
    int yMod = 1;
    int xReflect = 1;
    double xShift = 0;
    int headingMod = 0;
    int turnMod = 1;
    double tileWidth = 23.5;

    int RED   = 0;
    int BLUE  = 1;
    int LEFT  = 0;
    int RIGHT = 1;


    public int Team() {
        return team % 2;
    }

    public int Side() {
        return side % 2;
    }

    public int Mode() {
        return Mode % 4;
    }






    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    // UNITS ARE METERS
    double tagsize = 0.0381; // 1.5 inches

    int ID_TAG_OF_INTEREST  = 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 2; // Tag ID 2 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 3; // Tag ID 3 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field

        drive.setPoseEstimate(StartingPos());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!opModeIsActive() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        autoParkPosition = 0;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST2) {
                        autoParkPosition = 1;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST3) {
                        autoParkPosition = 2;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            drive.setPoseEstimate(StartingPos());
            //Gets inputs before init for gui
            initInputs();
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Detected: ", autoParkPosition);
            //Does Telemetry
            telemetryWhileInitialization();
            telemetry.update();
        }

        processPosition(); //Which numbers correlate to each side

        waitForStart();


        if (isStopRequested()) return;
        drive.setPoseEstimate(StartingPos());
        camera.stopStreaming();
        sleep(100);
        telemetry.update();
        drive.mainLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.mainLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.mainLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        Pose2d movement1 = new Pose2d(36 * xReflect, 60 * yMod, Math.toRadians(-90*yMod));
        Pose2d movement2 = new Pose2d(-34.5 * xReflect, 14 * yMod, Math.toRadians(-90 * yMod));
        Pose2d scorePos = new Pose2d(-29 * xReflect, 14 * yMod, Math.toRadians(180 + headingMod));    //Score Position
        Pose2d intakeStackPos = new Pose2d(-62.5 * xReflect, 15 * yMod, Math.toRadians(180 + headingMod));    //Intake cone stack Position
        Pose2d park1 = new Pose2d(-57.5 * xReflect, 13 * yMod, Math.toRadians(180 + headingMod));
        Pose2d park2 = new Pose2d(-34.5 * xReflect, 13 * yMod, Math.toRadians(180 + headingMod));
        Pose2d park3 = new Pose2d(-11.5 * xReflect, 13 * yMod, Math.toRadians(180 + headingMod));

        //Building trajectories

        //Park Only
        Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(movement2, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();

        Trajectory Park1 = drive.trajectoryBuilder(Movement1.end().plus(new Pose2d(0, 0, Math.toRadians(turnMod * 90))))
                .lineToLinearHeading(park1, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        //Trajectory Park2 = drive.trajectoryBuilder(Movement1.end().plus(new Pose2d(0, 0, Math.toRadians(turnMod*90))))
        //        .lineToLinearHeading(park2, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
        //                DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
        //        .build();
        Trajectory Park3 = drive.trajectoryBuilder(Movement1.end().plus(new Pose2d(0, 0, Math.toRadians(turnMod * 90))))
                .lineToLinearHeading(park3, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        //Cycle
        //Movement1
        Trajectory ScorePreloaded = drive.trajectoryBuilder(Movement1.end().plus(new Pose2d(0, 0, Math.toRadians(turnMod * 90))))
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory Intake1 = drive.trajectoryBuilder(ScorePreloaded.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory Score1 = drive.trajectoryBuilder(Intake1.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory Intake2 = drive.trajectoryBuilder(Score1.end())
                .lineToLinearHeading(intakeStackPos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory Score2 = drive.trajectoryBuilder(Intake2.end())
                .lineToLinearHeading(scorePos, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory CyclePark1 = drive.trajectoryBuilder(Score2.end())
                .lineToLinearHeading(park1, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory CyclePark2 = drive.trajectoryBuilder(Score2.end())
                .lineToLinearHeading(park2, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();
        Trajectory CyclePark3 = drive.trajectoryBuilder(Score2.end())
                .lineToLinearHeading(park3, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint((DriveConstants.MAX_ACCEL)))
                .build();

        drive.claw.setPosition(0.95);
        if (Mode() == 0) {//doing nothing

        } else if (Mode() == 1) {//cycling
            //Move to first pos
            drive.followTrajectory(Movement1);
            if (Side() == LEFT) {
                drive.turn(Math.toRadians(turnMod * 90));
            } else {
                drive.turn(Math.toRadians(turnMod * 90));
            }
            drive.followTrajectory(ScorePreloaded);
            //raise lift
            drive.mainLift.setTargetPosition(2500);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(1200);
            //turn table to junction
            if(Side() == LEFT) {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(-0.75);
                while(!isStopRequested() && opModeIsActive() && magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() && drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }else {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(0.75);
                while(!isStopRequested() && opModeIsActive() &&magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() &&drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }
            //drop cone
            sleep(400);
            drive.claw.setPosition(0);
            sleep(75);
            //turn table back to home
            if(Side()==0) {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(0.75);
                while(!isStopRequested() && opModeIsActive() &&magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() &&drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }else {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(-0.75);
                while(!isStopRequested() && opModeIsActive() && magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() && drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }
            //lower lift
            drive.mainLift.setTargetPosition(800);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

//            sleep(400);

            drive.followTrajectory(Intake1);
            //intake
            drive.mainLift.setTargetPosition(318);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(300);
            drive.claw.setPosition(0.95);
            sleep(600);

            //raise lift
            drive.mainLift.setTargetPosition(800);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(400);

            drive.followTrajectory(Score1);
            //raise lift
            drive.mainLift.setTargetPosition(2500);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(600);
            //turn table to junction
            if(Side()==0) {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(-0.75);
                while(!isStopRequested() && opModeIsActive() && magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() && drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }else {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(0.75);
                while(!isStopRequested() && opModeIsActive() &&magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() &&drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }
            //drop cone
            sleep(1000);
            drive.claw.setPosition(0);
            sleep(100);
            //turn table back to home
            if(Side()==0) {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(0.75);
                while(!isStopRequested() && opModeIsActive() &&magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() &&drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }else {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(-0.75);
                while(!isStopRequested() && opModeIsActive() && magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() && drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }
            //lower lift
            drive.mainLift.setTargetPosition(800);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

//            sleep(400);

            drive.followTrajectory(Intake2);
            //intake
            drive.mainLift.setTargetPosition(150);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(300);
            drive.claw.setPosition(0.95);
            sleep(600);

            //raise lift
            drive.mainLift.setTargetPosition(800);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(400);

            drive.followTrajectory(Score2);
            //raise lift
            drive.mainLift.setTargetPosition(2500);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            sleep(600);
            //turn table to junction
            if(Side()==0) {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(-0.75);
                while(!isStopRequested() && opModeIsActive() && magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() && drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }else {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(0.75);
                while(!isStopRequested() && opModeIsActive() &&magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() &&drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }
            //drop cone
            sleep(100);
            drive.claw.setPosition(0);
            sleep(200);
            //turn table back to home
            if(Side()==0) {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(0.75);
                while(!isStopRequested() && opModeIsActive() &&magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() &&drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }else {
                boolean magnetWasTouched = false;
                if(!drive.turnlimiter.getState()) {
                    magnetWasTouched = true;
                }
                drive.turntable.setPower(-0.75);
                while(!isStopRequested() && opModeIsActive() && magnetWasTouched) {
                    if(magnetWasTouched && drive.turnlimiter.getState()) {
                        magnetWasTouched = false;
                    }
                }
                while(!isStopRequested() && opModeIsActive() && drive.turnlimiter.getState()) {}
                drive.turntable.setPower(0);
            }
            //lower lift
            drive.mainLift.setTargetPosition(0);
            drive.mainLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.mainLift.setPower(0.75);

            switch (autoParkPosition) {
                case 1:
                    drive.followTrajectory(CyclePark2);
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (Side() == 0) {
                        drive.followTrajectory(CyclePark3);
                    } else {
                        drive.followTrajectory(CyclePark1);
                    }

                    if (isStopRequested()) return;
                    break;
                default:
                    if (Side() == 0) {
                        drive.followTrajectory(CyclePark1);
                    } else {
                        drive.followTrajectory(CyclePark3);
                    }
                    if (isStopRequested()) return;
                    break;
            }
        } else if (Mode() == 2) {//Just Parking
            drive.followTrajectory(Movement1);
            if (Side() == 0) {
                drive.turn(Math.toRadians(-90));
            } else {
                drive.turn(Math.toRadians(90));
            }

            switch (autoParkPosition) {
                case 1:
//                    drive.followTrajectory(Park2);
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (Side() == 0) {
                        drive.followTrajectory(Park3);
                    } else {
                        drive.followTrajectory(Park1);
                    }

                    if (isStopRequested()) return;
                    break;
                default:
                    if (Side() == 0) {
                        drive.followTrajectory(Park1);
                    } else {
                        drive.followTrajectory(Park3);
                    }
                    if (isStopRequested()) return;
                    break;
            }
        } else if (Mode() == 3) {//testing
            drive.followTrajectory(Movement1);
            if (Side() == 0) {
                drive.turn(Math.toRadians(turnMod * 90));
            } else {
                drive.turn(Math.toRadians(turnMod * 90));
            }
            drive.followTrajectory(ScorePreloaded);
            drive.followTrajectory(Intake1);
            drive.followTrajectory(Score1);
            drive.followTrajectory(Intake2);
            drive.followTrajectory(Score2);

            switch (autoParkPosition) {
                case 1:
                    drive.followTrajectory(CyclePark2);
                    if (isStopRequested()) return;
                    break;
                case 2:
                    if (Side() == 0) {
                        drive.followTrajectory(CyclePark3);
                    } else {
                        drive.followTrajectory(CyclePark1);
                    }

                    if (isStopRequested()) return;
                    break;
                default:
                    if (Side() == 0) {
                        drive.followTrajectory(CyclePark1);
                    } else {
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

    public Pose2d StartingPos() {
        double x, y, a;
        if (Team() == RED) {
            y = -63;
            a = 90;
            if (Side() == RIGHT) {
                x = 34.5;//23.25
            } else {
                x = -34.5;//23.25
            }
        } else {
            y = 63;
            a = -90;
            if (Side() == RIGHT) {
                x = -34.5;//23.25
            } else {
                x = 34.5;
            }
        }

        return new Pose2d(x, y, Math.toRadians(a));
    }

    public void telemetryWhileInitialization() {
        switch (Team()) {
            case (0):
                telemetry.addData("Team", "Red");
                switch (Side()) {
                    case (0):
                        telemetry.addData("Side", "Left");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Testing");
                                break;
                        }
                        break;
                    case (1):
                        telemetry.addData("Side", "Right");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Testing");
                                break;
                        }
                        break;
                }
                break;
            case (1):
                telemetry.addData("Team", "Blue");
                switch (Side()) {
                    case (0):
                        telemetry.addData("Side", "Left");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Testing");
                                break;
                        }
                        break;
                    case (1):
                        telemetry.addData("Side", "Right");
                        switch (Mode()) {
                            case (0):
                                telemetry.addData("Mode", "Nothing");
                                break;
                            case (1):
                                telemetry.addData("Mode", "Cycle");
                                break;
                            case (2):
                                telemetry.addData("Mode", "Park Only");
                                break;
                            case (3):
                                telemetry.addData("Mode", "Testing");
                                break;
                        }
                        break;
                }
                break;
        }


    }

    public void initInputs() {
        if (gamepad1.b && !button_b_was_pressed) {
            team++;
            button_b_was_pressed = true;
        } else if (!gamepad1.b && button_b_was_pressed) {
            button_b_was_pressed = false;
        }
        if (gamepad1.a && !button_a_was_pressed) {
            side++;
            button_a_was_pressed = true;
        } else if (!gamepad1.a && button_a_was_pressed) {
            button_a_was_pressed = false;
        }
        if (gamepad1.x && !button_x_was_pressed) {
            Mode++;
            button_x_was_pressed = true;
        } else if (!gamepad1.x && button_x_was_pressed) {
            button_x_was_pressed = false;
        }
    }

    public void processPosition() {
        if (team % 2 == 1) {
            yMod = 1;
        } else {
            yMod = -1;
        }
        if ((team % 2 == 0 && side % 2 == 0) || (team % 2 == 1 && side % 2 == 1)) {
            xReflect = 1;
            xShift = 0;
            headingMod = 180;
        } else if ((team % 2 == 0 && side % 2 == 1) || (team % 2 == 1 && side % 2 == 0)) {
            xReflect = -1;
            xShift = 70.5;
            headingMod = 0;
        }
        if (side % 2 == 0) {
            turnMod = -1;
        } else {
            turnMod = 1;
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
