package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Auto extends CameraTesting {
    boolean button_b_was_pressed = false;
    boolean button_a_was_pressed = false;
    boolean button_x_was_pressed = false;
    int team = 0;// 0 = red 1 = blue    which team we are on
    int side = 0;// 0 = left 1 = right  are we left or right
    int Mode = 0;//0 = nothing          are we just parking or otherwise?
    int autoParkPosition = 0;
    int numOfTrajs = 1;
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
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new SleeveOrientationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        Trajectory Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .build();
        Trajectory Movement2 = drive.trajectoryBuilder(Movement1.end())
                .build();
        Trajectory ScorePos1 = drive.trajectoryBuilder(Movement2.end())
                .build();
        Trajectory IntakeStackPos1 = drive.trajectoryBuilder(ScorePos1.end())
                .build();
        Trajectory ScorePos2 = drive.trajectoryBuilder(IntakeStackPos1.end())
                .build();
        Trajectory IntakeStackPos2 = drive.trajectoryBuilder(ScorePos2.end())
                .build();
        Trajectory ScorePos3 = drive.trajectoryBuilder(IntakeStackPos2.end())
                .build();
        Trajectory IntakeStackPos3 = drive.trajectoryBuilder(ScorePos3.end())
                .build();
        Trajectory ScorePos4 = drive.trajectoryBuilder(IntakeStackPos3.end())
                .build();
        Trajectory IntakeStackPos4 = drive.trajectoryBuilder(ScorePos4.end())
                .build();
        Trajectory ScorePos5 = drive.trajectoryBuilder(IntakeStackPos4.end())
                .build();
        Trajectory IntakeStackPos5 = drive.trajectoryBuilder(ScorePos5.end())
                .build();
        Trajectory ScorePos6 = drive.trajectoryBuilder(IntakeStackPos5.end())
                .build();
        Trajectory Park1 = drive.trajectoryBuilder(ScorePos6.end())
                .build();
        Trajectory Park2 = drive.trajectoryBuilder(ScorePos6.end())
                .build();
        Trajectory Park3 = drive.trajectoryBuilder(ScorePos6.end())
                .build();

        while (!opModeIsActive()) {


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
                }


                autoParkPosition = pipeline.getAnalysis();




            if (isStopRequested()) return;
            drive.setPoseEstimate(StartingPos());
            autoParkPosition = pipeline.getAnalysis();
            webcam.stopStreaming();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            sleep(100);
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
                xMod = 0;
                headingMod = 0;
            }else if (team % 2 == 0 && side % 2 == 1 || team % 2 == 1 && side % 2 == 0) {
                xMod = 72;
                headingMod = 180;
            }

            Pose2d movement1 = new Pose2d(12+xMod,60*yMod,Math.toRadians(90+headingMod));
            Pose2d movement2 = new Pose2d(12+xMod,12*yMod,Math.toRadians(180+headingMod));
            Pose2d scorePos = new Pose2d(24+xMod,12*yMod,Math.toRadians(180+headingMod));    //Score Position
            Pose2d intakeStackPos = new Pose2d(62+xMod,12*yMod,Math.toRadians(180+headingMod));    //Intake cone stack Position
            Pose2d park1 = new Pose2d(60+xMod,12*yMod,Math.toRadians(90+headingMod));
            Pose2d park2 = new Pose2d(36+xMod,12*yMod,Math.toRadians(90+headingMod));
            Pose2d park3 = new Pose2d(12+xMod,12*yMod,Math.toRadians(90+headingMod));
            Movement1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(movement1)
                    .build();
            Movement2 = drive.trajectoryBuilder(Movement1.end())
                    .lineToLinearHeading(movement2)
                    .build();
            ScorePos1 = drive.trajectoryBuilder(Movement2.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            IntakeStackPos1 = drive.trajectoryBuilder(ScorePos1.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            ScorePos2 = drive.trajectoryBuilder(IntakeStackPos1.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            IntakeStackPos2 = drive.trajectoryBuilder(ScorePos2.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            ScorePos3 = drive.trajectoryBuilder(IntakeStackPos2.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            IntakeStackPos3 = drive.trajectoryBuilder(ScorePos3.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            ScorePos4 = drive.trajectoryBuilder(IntakeStackPos3.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            IntakeStackPos4 = drive.trajectoryBuilder(ScorePos4.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            ScorePos5 = drive.trajectoryBuilder(IntakeStackPos4.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            IntakeStackPos5 = drive.trajectoryBuilder(ScorePos5.end())
                    .lineToLinearHeading(intakeStackPos)
                    .build();
            ScorePos6 = drive.trajectoryBuilder(IntakeStackPos5.end())
                    .lineToLinearHeading(scorePos)
                    .build();
            Park1 = drive.trajectoryBuilder(ScorePos6.end())
                    .lineToLinearHeading(park1)
                    .build();
            Park2 = drive.trajectoryBuilder(ScorePos6.end())
                    .lineToLinearHeading(park2)
                    .build();
            Park3 = drive.trajectoryBuilder(ScorePos6.end())
                    .lineToLinearHeading(park3)
                    .build();
        }

        if (Mode % 3 == 0) {//doing nothing

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

}
