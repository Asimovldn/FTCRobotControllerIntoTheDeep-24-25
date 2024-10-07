package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadInOuttake
{
    Gamepad gamepad;
    Outtake outtake;
    Intake intake;


    ElapsedTime switchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static int LOW_BASKET_POSITION = 2100; // Ticks
    public static int HIGH_BASKET_POSITION = 3000; // Ticks

    enum Take
    {
        INTAKE, OUTTAKE
    }

    Take currentTake = Take.OUTTAKE;

    public GamepadInOuttake(Gamepad gamepad, Outtake outtake /*Intake intake*/)
    {
        this.gamepad = gamepad;
        this.outtake = outtake;
        //this.intake = intake;

        switchTimer.reset();
    }

    public void update()
    {
        switch (currentTake)
        {
            case INTAKE:
                intakeControl();
                break;

            case OUTTAKE:
                outtakeControl();
                break;
        }
    }

    void intakeControl()
    {
        if (gamepad.left_bumper && switchTimer.time() > 0.5)
        {
            switchTimer.reset();
            currentTake = Take.OUTTAKE;
        }

        if (gamepad.dpad_down)
        {
            intake.MoveServo(Intake.IntakePosition.OUT);
        }

        if (gamepad.dpad_left)
        {
            intake.MoveServo(Intake.IntakePosition.HALF);
        }

        if (gamepad.dpad_up)
        {
            intake.MoveServo(Intake.IntakePosition.IN);
        }

        if (gamepad.b)
        {
            // Slide para frente
        }

        if (gamepad.a)
        {
            // Slide para tras
        }
    }

    void outtakeControl()
    {
        if (gamepad.left_bumper && switchTimer.time() > 0.5)
        {
            switchTimer.reset();
            currentTake = Take.INTAKE;
        }

        if (gamepad.y)
        {
            outtake.MoveSlide(LOW_BASKET_POSITION - outtake.getCurrentSlidePosition()[0]);
        } else if (gamepad.b)
        {
            outtake.MoveSlide(HIGH_BASKET_POSITION - outtake.getCurrentSlidePosition()[0]);
        } else if (gamepad.a)
        {
            outtake.MoveSlide(-outtake.getCurrentSlidePosition()[0]);
        } else {
            outtake.HoldPosition();
        }

        if (gamepad.dpad_left)
        {
            outtake.MoveServo(Outtake.OuttakePosition.IN);
        }

        if (gamepad.dpad_right)
        {
            outtake.MoveServo(Outtake.OuttakePosition.OUT);
        }
    }
}



/*package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gamepaddrive {

    double x;

    double y;

    double rotation;

    Gamepad gamepad;


    private MecannumDriveHandler Drive;

    enum Centric
    {
        ROBOT_CENTRIC, FIELD_CENTRIC
    }

    enum Reduction
    {
        STANDARD, HALF
    }

    Reduction CurrentReduction = Reduction.STANDARD;

    Centric CurrentCentric = Centric.FIELD_CENTRIC;

    ElapsedTime timerCentric = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime timerReduction = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public Gamepaddrive(Gamepad gamepad, MecannumDriveHandler Drive) {
        this.gamepad = gamepad;
        this.Drive = Drive;
        timerCentric.reset();
        timerReduction.reset();
    }

    public void update() {

        boolean changeCentric = gamepad.left_bumper;
        boolean changeReduction = gamepad.right_bumper;


        switch (CurrentReduction)
        {
            case STANDARD:
                if (changeReduction && timerReduction.time() > 0.5)
                {
                    CurrentReduction = Reduction.HALF;
                    timerReduction.reset();
                }
                x = gamepad.left_stick_x;
                y = -gamepad.left_stick_y;
                rotation = gamepad.right_stick_x;
                break;

            case HALF:
                if (changeReduction && timerReduction.time() > 0.5)
                {
                    CurrentReduction = Reduction.STANDARD;
                    timerReduction.reset();
                }
                x = gamepad.left_stick_x / 2;
                y = -gamepad.left_stick_y / 2;
                rotation = gamepad.right_stick_x / 2;
                break;
        }

        switch (CurrentCentric)
        {
            case ROBOT_CENTRIC:

                if (changeCentric && timerCentric.time() > 0.5)
                {
                    CurrentCentric = Centric.FIELD_CENTRIC;
                    timerCentric.reset();
                }
                Drive.Analog(x,y,rotation);

                break;

            case FIELD_CENTRIC:

                if (changeCentric && timerCentric.time() > 0.5)
                {
                    CurrentCentric = Centric.ROBOT_CENTRIC;
                    timerCentric.reset();
                }
                Drive.FieldCentric(x,y,rotation);

                break;
        }
    }

}
*/
