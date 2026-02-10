package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretPID {
    // PID coefficients (TUNE THESE)
    public double kP = 0.01;
    public double kI = 0.0;
    public double kD = 0.0005;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public double update(double target, double current) {
        double error = target - current;

        double dt = timer.seconds();
        timer.reset();

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}