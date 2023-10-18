package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class module {

    private DcMotor M2;
    private DcMotor M1;


    int MaxVelocity;

    double ErrorR;
    double Error2;
    double ModuleAngle0_360;
    double ErrorL;
    int C1;
    int C2;
    int ErrorBand;
    int DesiredAngleR;
    int DesiredAngleL;
    double ErrorCorrection;

public void hardwareMap() {
    M2 = hardwareMap.get(DcMotor.class, "M2");
    M1 = hardwareMap.get(DcMotor.class, "M1");
    M2.setDirection(DcMotor.Direction.REVERSE);


}

    public void driveToModule(double velocity, int DesiredAngle) {


        // Put initialization blocks here.
        C1 = 5;
        C2 = 0;
        MaxVelocity = 1;
        ErrorBand = 3;
        DesiredAngleR = DesiredAngle;
        DesiredAngleL = 360 - DesiredAngleR;
        ModuleAngle0_360 = (M1.getCurrentPosition() / 8192 - Math.floor(M1.getCurrentPosition() / 8192)) * 360;
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

        M1.setPower(velocity + ErrorCorrection);
        M2.setPower(velocity - ErrorCorrection);
        telemetry.addData("ModuleAngle0-360", ModuleAngle0_360);
        telemetry.addData("DesiredAngle", DesiredAngle);
        telemetry.addData("Error", Error2 * 360);
        telemetry.addData("ErrorR", ErrorR);
        telemetry.addData("ErrorCorrection", ErrorCorrection);
        telemetry.addData("Velocity", velocity);
        telemetry.update();
    }
}
