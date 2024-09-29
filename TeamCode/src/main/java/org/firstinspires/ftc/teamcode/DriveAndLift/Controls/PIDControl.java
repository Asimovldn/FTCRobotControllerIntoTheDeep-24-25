package org.firstinspires.ftc.teamcode.DriveAndLift.Controls;

public class PIDControl
{
    double kp;
    double ki;
    double kd;

    double proporcional = 0;
    double integral = 0;
    double derivative = 0;

    double lastError = 0;

    public PIDControl(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double setPoint, double ref)
    {
        double error = setPoint - ref;

        proporcional = error;

        integral += integral;

        derivative = error - lastError;

        lastError = error;

        return proporcional * kp + integral * ki + derivative * kd;
    }
}
