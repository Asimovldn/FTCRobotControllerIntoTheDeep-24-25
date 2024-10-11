package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.Gamepaddrive;
import org.firstinspires.ftc.teamcode.Auxiliar.MecannumDriveHandler;

@TeleOp
public class TesteControle extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap,telemetry, this);
        Gamepaddrive gamepadDrive = new Gamepaddrive(gamepad1, drive);
        waitForStart();

        while(opModeIsActive())
        {
            gamepadDrive.update();
        }
    }
}
