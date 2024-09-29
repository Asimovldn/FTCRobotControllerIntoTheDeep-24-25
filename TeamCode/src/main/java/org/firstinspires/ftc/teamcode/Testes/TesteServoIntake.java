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

          double intakePower = 0.0;

          while (opModeIsActive())
          {
              if (gamepad1.a)
              {
                  intake.MoveServo(Intake.IntakePosition.IN);
              }

              if (gamepad1.b)
              {
                  intake.MoveServo(Intake.IntakePosition.OUT);
              }

              if (gamepad1.y)
              {
                  intakePower = 1.0;
              }

              if (gamepad1.x)
              {
                  intakePower = 0.0;
              }

              if (gamepad1.left_bumper)
              {
                  intakePower = -1.0;
              }

              intake.setIntakePower(intakePower);
          }
    }
}
