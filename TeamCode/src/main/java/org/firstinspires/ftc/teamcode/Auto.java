package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous
public class Auto extends CameraTesting {
    boolean button_b_was_pressed = false; //Team
    boolean button_a_was_pressed = false; //Side
    boolean button_x_was_pressed = false; //Mode
    int team = 0;// 0 = red 1 = blue    which team we are on
    int side = 0;// 0 = left 1 = right  are we left or right
    int Mode = 0;//0 = nothing          are we just parking or otherwise?
    int autoParkPosition = 0;
    int yMod = 1;
    int xMod = 0;
    int headingMod = 0;
    public int Team() {
        return team % 2;
    }

    public int Side() {
        return side % 2;
    }

    public int Mode() {
        return Mode % 3;
    }
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family
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
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(10, 15, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!opModeIsActive()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        autoParkPosition = 0;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == ID_TAG_OF_INTEREST2)
                    {
                        autoParkPosition = 1;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == ID_TAG_OF_INTEREST3)
                    {
                        autoParkPosition = 2;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }


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
            switch (Team()){
                case (0):
                    telemetry.addData("Team", "Red");
                    switch (Side()){
                        case(0):
                            telemetry.addData("Side", "Left");
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    break;
                            }
                            break;
                        case(1):
                            telemetry.addData("Side", "Right");
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    break;
                            }
                            break;
                    }
                    break;
                case (1):
                    telemetry.addData("Team", "Blue");
                    switch (Side()){
                        case(0):
                            telemetry.addData("Side", "Left");
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    break;
                            }
                            break;
                        case(1):
                            telemetry.addData("Side", "Right");
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    break;
                            }
                            break;
                    }
                    break;
            }






            if (isStopRequested()) return;
            drive.setPoseEstimate(StartingPos());
            telemetry.addData("Detection = ", autoParkPosition);
            telemetry.update();

            /*
            switch (Team()){
                case (0):
                    telemetry.addData("Team", "Red");
                    telemetry.update();
                    switch (Side()){
                        case(0):
                            telemetry.addData("Side", "Left");
                            telemetry.update();
                            switch(Mode()){
                                case 0:
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case 1:
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case 2:
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                        case(1):
                            telemetry.addData("Side", "Right");
                            telemetry.update();
                            switch(Mode()){
                                case 0:
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case 1:
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case 2:
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                    }
                    break;
                case (1):
                    telemetry.addData("Team", "Blue");
                    telemetry.update();
                    switch (Side()){
                        case(0):
                            telemetry.addData("Side", "Left");
                            telemetry.update();
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                        case(1):
                            telemetry.addData("Side", "Right");
                            telemetry.update();
                            switch(Mode()){
                                case(0):
                                    telemetry.addData("Mode", "Nothing");
                                    telemetry.update();
                                    break;
                                case(1):
                                    telemetry.addData("Mode", "Cycle");
                                    telemetry.update();
                                    break;
                                case(2):
                                    telemetry.addData("Mode", "Park Only");
                                    telemetry.update();
                                    break;
                            }
                            break;
                    }
                    break;
            }*/

            if (team % 2 == 1) {
                yMod = -1;
            }else {
                yMod = 1;
            }
            if (team % 2 == 0 && side % 2 == 0 || team % 2 == 1 && side % 2 == 1) {
                xMod = 1;
                headingMod = 1;
            }else if (team % 2 == 0 && side % 2 == 1 || team % 2 == 1 && side % 2 == 0) {
                xMod = -1;
                headingMod = -1;
            }
        }

        //webcam.stopStreaming();

        Pose2d movement1 = new Pose2d(-12*xMod,-60*yMod,Math.toRadians(90));
        Pose2d movement2 = new Pose2d(-12*xMod,-12*yMod,Math.toRadians(180*headingMod));
        Pose2d scorePos = new Pose2d(-24*xMod,-12*yMod,Math.toRadians(180*headingMod));    //Score Position
        Pose2d intakeStackPos = new Pose2d(-62+xMod,-12*yMod,Math.toRadians(180*headingMod));    //Intake cone stack Position
        Pose2d park1 = new Pose2d(-60*xMod,-12*yMod,Math.toRadians(180*headingMod));
        Pose2d park2 = new Pose2d(-36*xMod,-12*yMod,Math.toRadians(180*headingMod));
        Pose2d park3 = new Pose2d(-12*xMod,-12*yMod,Math.toRadians(180*headingMod));

        Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(movement1)
                .build();
        Trajectory Movement2 = drive.trajectoryBuilder(Movement1.end())
                .lineToLinearHeading(movement2)
                .build();
        Trajectory ScorePos1 = drive.trajectoryBuilder(Movement2.end())
                .lineToLinearHeading(scorePos)
                .build();
        Trajectory IntakeStackPos1 = drive.trajectoryBuilder(ScorePos1.end())
                .lineToLinearHeading(intakeStackPos)
                .build();
        Trajectory ScorePos2 = drive.trajectoryBuilder(IntakeStackPos1.end())
                .lineToLinearHeading(scorePos)
                .build();
        Trajectory IntakeStackPos2 = drive.trajectoryBuilder(ScorePos2.end())
                .lineToLinearHeading(intakeStackPos)
                .build();
        Trajectory ScorePos3 = drive.trajectoryBuilder(IntakeStackPos2.end())
                .lineToLinearHeading(scorePos)
                .build();
        Trajectory IntakeStackPos3 = drive.trajectoryBuilder(ScorePos3.end())
                .lineToLinearHeading(intakeStackPos)
                .build();
        Trajectory ScorePos4 = drive.trajectoryBuilder(IntakeStackPos3.end())
                .lineToLinearHeading(scorePos)
                .build();
        Trajectory IntakeStackPos4 = drive.trajectoryBuilder(ScorePos4.end())
                .lineToLinearHeading(intakeStackPos)
                .build();
        Trajectory ScorePos5 = drive.trajectoryBuilder(IntakeStackPos4.end())
                .lineToLinearHeading(scorePos)
                .build();
        Trajectory IntakeStackPos5 = drive.trajectoryBuilder(ScorePos5.end())
                .lineToLinearHeading(intakeStackPos)
                .build();
        Trajectory ScorePos6 = drive.trajectoryBuilder(IntakeStackPos5.end())
                .lineToLinearHeading(scorePos)
                .build();
        Trajectory Park1 = drive.trajectoryBuilder(ScorePos6.end())
                .lineToLinearHeading(park1)
                .build();
        Trajectory Park2 = drive.trajectoryBuilder(ScorePos6.end())
                .lineToLinearHeading(park2)
                .build();
        Trajectory Park3 = drive.trajectoryBuilder(ScorePos6.end())
                .lineToLinearHeading(park3)
                .build();

        if (Mode % 3 == 0) {//doing stuff to get it to work

            Trajectory forward = drive.trajectoryBuilder(ScorePos6.end())
                    .forward(46)
                    .build();

            drive.claw.setPosition(0.150);
            drive.followTrajectory(forward);
        }else if (Mode % 3 == 1) {//cycling
            drive.followTrajectory(Movement1);
            drive.followTrajectory(Movement2);
            drive.followTrajectory(ScorePos1);
            //Score Cone
            drive.followTrajectory(IntakeStackPos1);
            //Intake Cone
            drive.followTrajectory(ScorePos2);
            //Score Cone
            drive.followTrajectory(IntakeStackPos2);
            //Intake Cone
            drive.followTrajectory(ScorePos3);
            //Score Cone
            drive.followTrajectory(IntakeStackPos3);
            //Intake Cone
            drive.followTrajectory(ScorePos4);
            //Score Cone
            drive.followTrajectory(IntakeStackPos4);
            //Intake Cone
            drive.followTrajectory(ScorePos5);
            //Score Cone
            drive.followTrajectory(IntakeStackPos5);
            //Intake Cone
            drive.followTrajectory(ScorePos6);
            //Score Cone

            switch(autoParkPosition){
                case 1:
                    drive.followTrajectory(Park2);
                    if(isStopRequested())return;
                    break;
                case 2:
                    drive.followTrajectory(Park1);
                    if(isStopRequested()) return;
                    break;
                default:
                    drive.followTrajectory(Park3);
                    if(isStopRequested()) return;
                    break;


            }
        }else if (Mode % 3 == 2) {//Just Parking
            drive.followTrajectory(Movement1);
            drive.followTrajectory(Movement2);

            switch(autoParkPosition){
                case 1:
                    drive.followTrajectory(Park2);
                    if(isStopRequested())return;
                    break;
                case 2:
                    drive.followTrajectory(Park1);
                    if(isStopRequested()) return;
                    break;
                default:
                    drive.followTrajectory(Park3);
                    if(isStopRequested()) return;
                    break;


            }
        }

        PoseStorage.team = team % 2;
    }



    public Pose2d StartingPos() {
        double x, y, a;
        if (team == 0) {
            y = -63.75;
            a = 90;
            if (side == 1) {
                x = 36;
            } else {
                x = -36;

            }
        } else {
            y = 63.75;
            a = -90;
            if (side == 1) {
                x = 36;

            } else {
                x = -36;
            }
        }

        return new Pose2d(x, y, Math.toRadians(a));
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
