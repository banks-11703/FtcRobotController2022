package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "DriveCodeNotBryce", group = "Linear Opmode")
public class DriveCodeNotBryce extends DriveCodeCommonNotBryce {
    public void runOpMode() {
        Initialization();
        if (isStopRequested()) return;
        autoHome = true;
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            Lift();
            tipCones();
            Claw();
            TurnTable();
            ShootOut();
            Telemetry();
            ColorSensor();
            Lights();
        }
    }
}