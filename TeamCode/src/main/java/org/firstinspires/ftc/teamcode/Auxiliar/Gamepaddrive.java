package org.firstinspires.ftc.teamcode.Auxiliar;

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
