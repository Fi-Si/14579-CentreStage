package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDCorrector {
    public DcMotorEx motor;
    public static double kP = 1;
    public static double kI = 0.1;
    public static double kD = 0.01;
    public static double kF = 0;
    public double output = 0;

    boolean correctsvelocity = false;

    private double integralSum = 0;
    private double lasterror = 0;

    ElapsedTime timer = new ElapsedTime();

    public PIDCorrector(double kP, double kI, double kD, double kF, boolean correctsvelocity) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        if (correctsvelocity) {this.kF = kF;}
        this.correctsvelocity = correctsvelocity;
    }

    public void kPIDUpdate(double kP, double kI, double kD) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }

    public void kPIDFUpdate(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kF = kF;
    }

    public double UpatedCorrection(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lasterror) * timer.seconds();
        lasterror = error;
        timer.reset();
        if (correctsvelocity) {
            output = (error * kP) + (integralSum * kI) + (derivative * kD) + (error*kF);
        }
        else {
            output = (error * kP) + (integralSum * kI) + (derivative * kD);
        }
        return output;
    }
}
