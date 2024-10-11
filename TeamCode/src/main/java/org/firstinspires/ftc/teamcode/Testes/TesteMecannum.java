package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.MecannumDriveHandler;

@TeleOp

public class TesteMecannum extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap, telemetry, this);
        waitForStart();

        while (opModeIsActive())
        {
            drive.FieldCentric(gamepad1.left_stick_x / 2, -gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);
        }
    }
}
