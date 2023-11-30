package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

public class PErrorAdjustment {

    public double ErrorR;

    public double ErrorL;
    public double Error2;
    public double ErrorCorrection;
    public double Error22;z


    public static double ErrorBand = 4;
    public static double C1 = 0;
    public static double C2 = 3;
    public double DesiredAngle;
    public DcMotorEx EncoderA;
    public DcMotorEx EncoderB;
    public ModuleSide moduleside;


    public PErrorAdjustment(double DesiredAngle, DcMotorEx EncoderA, DcMotorEx EncoderB, ModuleSide moduleside) {
        this.DesiredAngle = DesiredAngle;
        this.EncoderA = EncoderA;
        this.EncoderB = EncoderB;
        this.moduleside = moduleside;
    }

    public double PErrorAdjuster() {
        if (moduleside == ModuleSide.RIGHT) {
            double DesiredAngleR = DesiredAngle;
            double DesiredAngleL = 360 - DesiredAngleR;
            double ModuleAngle0_360 = (this.EncoderA.getCurrentPosition() / 8192 - Math.floor(this.EncoderA.getCurrentPosition() / 8192)) * 360;
            if (DesiredAngleR > ModuleAngle0_360) {
                ErrorR = DesiredAngleR - ModuleAngle0_360;
            } else {
                ErrorR = 360 - (ModuleAngle0_360 - DesiredAngleR);
            }
            ErrorL = -(360 - ErrorR);
            if (Math.abs(ErrorR) < ErrorBand || Math.abs(ErrorL) < ErrorBand) {
                Error2 = 0;
            } else if (Math.abs(ErrorR) < Math.abs(ErrorL)) {
                Error2 = ErrorR / 360;
            } else {
                Error2 = ErrorL / 360;
            }
            ErrorCorrection = Error2 * C1 + C2;
            return ErrorCorrection;
        }
        else {
            DesiredAngleR2 = DesiredAngle;
            DesiredAngleL2 = 360 - DesiredAngleR2;
            ModuleAngle0_3602 = (EncoderB.getCurrentPosition() / 8192 - Math.floor(EncoderB.getCurrentPosition() / 8192)) * 360;
            if (DesiredAngleR2 > ModuleAngle0_3602) {
                ErrorR2 = DesiredAngleR2 - ModuleAngle0_3602;
            } else {
                ErrorR2 = 360 - (ModuleAngle0_3602 - DesiredAngleR2);
            }
            ErrorL2 = -(360 - ErrorR2);
            if (Math.abs(ErrorR2) < ErrorBand || Math.abs(ErrorL2) < ErrorBand) {
                Error22 = 0;
            } else if (Math.abs(ErrorR2) < Math.abs(ErrorL2)) {
                Error22 = ErrorR2 / 360;
            } else {
                Error22 = ErrorL2 / 360;
            }
            ErrorCorrection2 = Error22 * C1 + C2;
        }

