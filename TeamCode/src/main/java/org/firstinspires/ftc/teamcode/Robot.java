package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    DriveController driveController;
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpMode opMode;
    DcMotorEx climb1, climb2, Lift1, Lift2;
    CRServo IntakeL, IntakeR;
    Servo Outtake, Plane, Diff1, Diff2;


    public Robot (OpMode opMode, boolean isAuto) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        climb1 = hardwareMap.get(DcMotorEx.class, "EncoderA");
        climb2 = hardwareMap.get(DcMotorEx.class, "EncoderB");
        Diff2 = hardwareMap.get(Servo.class, "Diff2");
        Outtake = hardwareMap.get(Servo.class, "Outtake");
        Plane = hardwareMap.get(Servo.class, "Plane");
        Diff1 = hardwareMap.get(Servo.class, "Diff1");
        IntakeL = hardwareMap.get(CRServo.class, "IntakeL");
        Lift1 = hardwareMap.get(DcMotorEx.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotorEx.class, "Lift2");
        IntakeR = hardwareMap.get(CRServo.class, "IntakeR");
        driveController = new DriveController(this);
    }


    public void initIMU () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    public Angle getRobotHeading () {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return new Angle(-heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    public void wait (int millis, LinearOpMode linearOpMode) {
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime && linearOpMode.opModeIsActive()) {}
    }
}