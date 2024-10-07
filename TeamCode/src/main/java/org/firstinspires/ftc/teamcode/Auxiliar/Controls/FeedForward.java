package org.firstinspires.ftc.teamcode.Auxiliar.Controls;

public class FeedForward
{
    double ka;
    double kv;
    double ks;
    double kg;

    public FeedForward(double kv, double ka, double ks)
    {
        this.kv = kv;
        this.ka = ka;
        this.ks = ks;
    }

    public FeedForward(double kv, double ka, double ks, double kg)
    {
        this.kv = kv;
        this.ka = ka;
        this.ks = ks;
        this.kg = kg;
    }

    public double calculate(double vSetpoint, double aSetPoint, int direction)
    {
        return ks * direction + vSetpoint * kv + aSetPoint * ka;
    }

    public double calculate(double vSetpoint, double aSetpoint, double gFactor, int direction)
    {
        return ks * direction + vSetpoint * kv + aSetpoint * ka + gFactor * kg;
    }
}