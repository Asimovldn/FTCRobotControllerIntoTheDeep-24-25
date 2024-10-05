package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.MecannumDriveHandler;

@TeleOp

public class TesteMecannum extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap);
        waitForStart();

        while (opModeIsActive())
        {
            drive.FielCentric(gamepad1.left_stick_x / 2, -gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);
        }
    }
}
