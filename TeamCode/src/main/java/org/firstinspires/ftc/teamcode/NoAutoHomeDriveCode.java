package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NoAutoHomeDriveCode extends DriveCodeCommonNoAutoHome {
    @Override
    public void runOpMode() {
        Initialization();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            RawDriving();
            Toggles();
            if(button_a2_was_pressed || button_b2_was_pressed) {
                Lift();
            }
            Claw();
            TurnTable();
            ShootOut();
            Telemetry();
//            ColorSensor();
        }
    }
}