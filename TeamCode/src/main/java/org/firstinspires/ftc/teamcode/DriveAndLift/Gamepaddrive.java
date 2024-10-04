package org.firstinspires.ftc.teamcode.DriveAndLift;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Gamepaddrive {

    double x;

    double y;

    double Rotacao;

    Gamepad gamepad;

    boolean reducted = false;

    private MecannumDriveHandler Drive;

    public Gamepaddrive(Gamepad gamepad, MecannumDriveHandler Drive) {
        this.gamepad = gamepad;
        this.Drive = Drive;
    }

    public void update() {


        x = gamepad.left_stick_x;
        y = gamepad.left_stick_y;
        Rotacao = gamepad.right_stick_x;

        Drive.Analog(x, y, Rotacao);

        if (reducted)
        {
            x = gamepad.left_stick_x;
            y = gamepad.left_stick_y;
            Rotacao = gamepad.right_stick_x;

            Drive.Analog(x / 2, y / 2, Rotacao / 2);

        }else {

            x = gamepad.left_stick_x;
            y = gamepad.left_stick_y;
            Rotacao = gamepad.right_stick_x;

            Drive.Analog(x, y, Rotacao);

        }
    }

}
