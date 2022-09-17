package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.CameraTesting;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Auto extends CameraTesting {
    boolean button_b_is_pressed = gamepad1.b;
    boolean button_b_was_pressed;
    boolean button_a_is_pressed = gamepad1.a;
    boolean button_a_was_pressed;
    boolean button_x_is_pressed = gamepad1.x;
    boolean button_x_was_pressed;
    int team = 0;// 0 = red 1 = blue
    int side = 0;// 0 = left 1 = right
    int mode = 0;//0 = nothing

    public int Team() {
        return team % 2;
    }

    public int Side() {
        return side % 2;
    }

    public int Mode() {
        return mode % 3;
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
        while (!opModeIsActive()) {
            if (button_b_is_pressed && !button_b_was_pressed) {
                team++;
                button_b_was_pressed = true;
            } else if (!button_b_is_pressed && button_b_was_pressed) {
                button_b_was_pressed = false;
            }
            if (button_a_is_pressed && !button_a_was_pressed) {
                side++;
                button_a_was_pressed = true;
            } else if (!button_a_is_pressed && button_a_was_pressed) {
                button_a_was_pressed = false;
            }
            if (button_x_is_pressed && !button_x_was_pressed) {
                mode++;
                button_x_was_pressed = true;
            } else if (!button_x_is_pressed && button_x_was_pressed) {
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
            waitForStart();

            if (isStopRequested()) return;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            sleep(100);
            telemetry.update();
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

            // Transfer the current pose to PoseStorage so we can use it in TeleOp
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}
