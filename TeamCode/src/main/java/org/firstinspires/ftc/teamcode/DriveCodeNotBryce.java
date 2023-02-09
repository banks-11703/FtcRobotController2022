package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriveCodeNotBryce", group = "Linear Opmode")
public class DriveCodeNotBryce extends DriveCodeCommonNotBryce {
    @Override
    public void runOpMode() {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            Lift();
            Claw();
            TurnTable();
            ShootOut();
            Telemetry();
            ColorSensor();
        }
    }
}