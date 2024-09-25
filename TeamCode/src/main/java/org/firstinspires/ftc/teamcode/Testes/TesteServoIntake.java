package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.Intake;

@TeleOp
public class TesteServoIntake extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
          Intake intake = new Intake(hardwareMap);

          waitForStart();

          while (opModeIsActive())
          {
              if (gamepad1.a)
              {
                  intake.Move(Intake.IntakePosition.IN);
              }

              if (gamepad1.b)
              {
                  intake.Move(Intake.IntakePosition.OUT);
              }
          }
    }
}
