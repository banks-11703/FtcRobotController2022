package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class DriveCode extends DriveCodeCommon {
    @Override
    public void runOpMode() {
        Initialization();
        if (isStopRequested()) return;
        autoHome = true;
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            Lift();
            Claw();
            TurnTable();
            ShootOut();
            Telemetry();
            ColorSensor(false);
            Lights();
        }
    }
}