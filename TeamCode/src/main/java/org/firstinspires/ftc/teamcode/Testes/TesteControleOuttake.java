package org.firstinspires.ftc.teamcode.Testes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.GamepadInOuttake;
import org.firstinspires.ftc.teamcode.Auxiliar.Gamepaddrive;
import org.firstinspires.ftc.teamcode.Auxiliar.Intake;
import org.firstinspires.ftc.teamcode.Auxiliar.MecannumDriveHandler;
import org.firstinspires.ftc.teamcode.Auxiliar.Outtake;

@TeleOp
public class TesteControleOuttake extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        Outtake outtake = new Outtake(hardwareMap, this, telemetry);
        Intake intake = new Intake(hardwareMap);
        GamepadInOuttake gamepadInOuttake = new GamepadInOuttake(gamepad1, outtake, intake);
        Gamepaddrive gamepaddrive = new Gamepaddrive(gamepad1, new MecannumDriveHandler(hardwareMap));

        waitForStart();

        while (opModeIsActive())
        {
            gamepadInOuttake.update(gamepaddrive);
        }
    }
}
