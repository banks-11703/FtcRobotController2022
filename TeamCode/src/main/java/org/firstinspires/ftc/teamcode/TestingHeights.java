package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class TestingHeights extends DriveCodeCommonNotBryce {
    String mode = "teleOp";
    int tPos = 0;
    //p=10,i=0.049988,d=0,f=0
    public static double p = 10;//10
    public static double I = 0.049988;//0.049988
    public static double d = 0;//0
    public static double f = 0;//0
    boolean closed = false;
    double pos = 0;
    boolean wasChanged = false;
    double[] coneHeights = {0,0.033,0.0480,0.0720,0.0870,0.0930};
    double[] coneHeightsClear = {0.0510,0.069,0.104444,0.1400,0.1620,0.1640};
    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            RawDriving();
            Toggles();

            if(button_a_was_pressed) {
                pos += 0.001;
            } else if(button_x_was_pressed) {
                pos -= 0.001;
            }
            if(button_dpaddown1_was_pressed && !wasChanged) {
                if(closed) {
                    SClaw(true);
                    closed = false;
                } else {
                    SClaw(false);
                    closed = true;
                }
                wasChanged = true;
            }
            if(!button_dpaddown1_was_pressed && wasChanged) {
                wasChanged = false;
            }
            drive.slift.setPosition(pos);


            telemetry.addData("slift pos",drive.slift.getPosition());
            telemetry.update();


        }
    }
}