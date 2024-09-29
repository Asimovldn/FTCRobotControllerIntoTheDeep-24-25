package org.firstinspires.ftc.teamcode.DriveAndLift.Controls;

public class TrapezoidalMotionProfile
{
    double maxVel;
    double maxAccel;
    double distance;

    double accelDistance;
    double cruiseDistance;

    double endAccelTimeStamp;
    double endCruiseTimeStamp;
    double totalTime;

    public boolean isBusy;

    public TrapezoidalMotionProfile(double maxVel, double maxAccel, double distance)
    {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.distance = distance;

        double timeToAccel = this.maxVel / maxAccel;

        accelDistance = 0.5 * maxAccel * Math.pow(timeToAccel, 2);

        if (accelDistance > distance / 2)
        {
            timeToAccel = Math.sqrt(distance / maxAccel);
        }

        accelDistance = 0.5 * maxAccel * Math.pow(timeToAccel, 2);

        this.maxVel = maxAccel * timeToAccel;

        cruiseDistance = distance - 2 * accelDistance;
        double cruiseTime = cruiseDistance / this.maxVel;

        endAccelTimeStamp = timeToAccel;
        endCruiseTimeStamp = timeToAccel + cruiseTime;

        totalTime = 2 * timeToAccel + cruiseTime;


    }


    // Retorna na ordem: distancia atual, velocidade atual, aceleração atual
    public double[] calculateMotionProfile(double time)
    {
        if (time < totalTime)
        {
            isBusy = true;
        }

        if (!isBusy)
        {
            return new double[] {0,0,0};
        }

        double currentDistance = 0.0;
        double currentVelocity = 0.0;
        double currentAccel = 0.0;

        if (time < endAccelTimeStamp)
        {
            // Acelerando
            currentAccel = maxAccel;
            currentVelocity = maxAccel * time;
            currentDistance = 0.5 * maxAccel * Math.pow(time, 2);
        } else if (time < endCruiseTimeStamp) {
            // Cruzeiro
            currentAccel = 0.0;
            currentVelocity = maxVel;
            currentDistance = accelDistance + maxVel * (time - endAccelTimeStamp);
        } else if (time < totalTime) {
            // Desacelerando
            currentAccel = -maxAccel;
            currentVelocity = maxVel - maxAccel * (time - endCruiseTimeStamp);
            currentDistance = accelDistance + cruiseDistance + maxVel * (time - endCruiseTimeStamp)
                    - 0.5 * maxAccel * Math.pow(time - endCruiseTimeStamp, 2);
        }

        return new double[] {currentDistance, currentVelocity, currentAccel};
    }

}
