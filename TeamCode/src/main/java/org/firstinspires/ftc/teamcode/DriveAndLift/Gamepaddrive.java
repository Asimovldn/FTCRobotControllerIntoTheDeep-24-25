package org.firstinspires.ftc.teamcode.DriveAndLift;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Gamepaddrive {

    double x;

    double y;

    double Rotacao;

    Gamepad gamepad;

    boolean reducted = false;

    private MecannumDriveHandler Drive;

    enum Centric
    {
        ROBOT_CENTRIC, FIELD_CENTRIC
    };

    Centric CurrentCentric = Centric.FIELD_CENTRIC;

    public Gamepaddrive(Gamepad gamepad, MecannumDriveHandler Drive) {
        this.gamepad = gamepad;
        this.Drive = Drive;
    }

    public void update() {


        x = gamepad.left_stick_x;
        y = gamepad.left_stick_y;
        Rotacao = gamepad.right_stick_x;

        if (reducted)
        {
            x /= 2;
            y /= 2;
            Rotacao /= 2;
        }

        switch (CurrentCentric)
        {
            case ROBOT_CENTRIC:
                Drive.Analog(x,y,Rotacao);
                break;

            case FIELD_CENTRIC:
            //    Drive.FieldCentric(x,y,Rotacao);
                break;
        }
    }

}
