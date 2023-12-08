package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Autonomous(name = "CloseRedAlliancePurpleYellowLeftPark (Blocks to Java)")
public class CloseRedAlliancePurpleYellowLeftPark extends LinearOpMode {

    private DcMotor M1;
    private DcMotor EncoderA;
    private DcMotor R2a;
    private DcMotor EncoderB;
    private Servo Outtake;
    private DcMotor M2;
    private DcMotor R2b;
    private BNO055IMU imu;
    private Servo Diff2;
    private Servo Plane;
    private Servo Diff1;
    private CRServo IntakeL;
    private DcMotor Lift1;
    private DcMotor Lift2;
    private CRServo IntakeR;

    int RobotHeadingChange;
    double Diff1Rest;
    double Diff2Rest;
    float Yaw;
    float RightStickX;
    int LeftTrigger;
    double RotationAuthority;
    int DesiredAngle;
    int EncoderDistA;
    float RotationModify2;
    int EncoderDistB;
    int C1;
    int C2;
    int ErrorBand;
    double SpeedOut;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        VisionProcessor csVisionProcessor;
        VisionPortal myVisionPortal;
        int MaxVelocity;

        M1 = hardwareMap.get(DcMotor.class, "M1");
        EncoderA = hardwareMap.get(DcMotor.class, "EncoderA");
        R2a = hardwareMap.get(DcMotor.class, "R2a");
        EncoderB = hardwareMap.get(DcMotor.class, "EncoderB");
        Outtake = hardwareMap.get(Servo.class, "Outtake");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        R2b = hardwareMap.get(DcMotor.class, "R2b");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Diff2 = hardwareMap.get(Servo.class, "Diff2");
        Plane = hardwareMap.get(Servo.class, "Plane");
        Diff1 = hardwareMap.get(Servo.class, "Diff1");
        IntakeL = hardwareMap.get(CRServo.class, "IntakeL");
        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        IntakeR = hardwareMap.get(CRServo.class, "IntakeR");

        // Custom CenterStage Vision Processor
        csVisionProcessor = CSVisionProcessor.getCSVision(213, 0, 267, 213, 267, 426, 267);
        // Create a VisionPortal, with the specified webcam name and AprilTag processor, and assign it to a variable.
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam1"), csVisionProcessor);
        while (!isStarted() && !opModeIsActive()) {
            // Returns the current starting postion
            telemetry.addData("Position", CSVisionProcessor.getPosition());
            telemetry.update();
        }
        waitForStart();
        IMU2();
        LiftAndDiffSetup();
        M1.setDirection(DcMotor.Direction.REVERSE);
        EncoderA.setDirection(DcMotor.Direction.REVERSE);
        R2a.setDirection(DcMotor.Direction.REVERSE);
        EncoderB.setDirection(DcMotor.Direction.REVERSE);
        C1 = 3;
        C2 = 0;
        MaxVelocity = 1;
        ErrorBand = 3;
        if (opModeIsActive()) {
            // Returns the current starting postion as an integer
            if (CSVisionProcessor.getIntPosition() == 1) {
                Outtake.setPosition(0);
                RobotHeadingChange = 0;
                sleep(200);
                Strafe(1200, 350, 0.25);
                RobotHeadingChange = 45;
                sleep(500);
                Strafe(1300, 0, 0.2);
                sleep(500);
                Strafe(900, 225, 0.25);
                RobotHeadingChange = -90;
                sleep(500);
                Strafe(3000, 0, 0.2);
                sleep(500);
                Strafe(450, 90, 0.25);
                LiftPositionUP(-400);
                sleep(500);
                Strafe(200, 0, 0.15);
                Outtake.setPosition(1);
                sleep(500);
                Strafe(200, 180, 0.25);
                sleep(500);
                Strafe(1400, 90, 0.25);
                sleep(500);
                Strafe(900, 0, 0.25);
                requestOpModeStop();
            }
            // Returns the current starting postion as an integer
            if (CSVisionProcessor.getIntPosition() == 2) {
                Outtake.setPosition(0);
                RobotHeadingChange = 0;
                sleep(200);
                Strafe(2200, 0, 0.25);
                LiftPositionUP(-50);
                sleep(500);
                Strafe(1500, 200, 0.25);
                sleep(500);
                Strafe(2000, 278, 0.3);
                sleep(500);
                RobotHeadingChange = -90;
                Strafe(1300, 0, 0.25);
                sleep(500);
                Strafe(500, 90, 0.25);
                LiftPositionUP(-400);
                sleep(500);
                Strafe(300, 0, 0.15);
                Outtake.setPosition(1);
                sleep(500);
                Strafe(300, 180, 0.25);
                sleep(500);
                Strafe(600, 90, 0.25);
                sleep(500);
                Strafe(700, 90, 0.25);
                sleep(500);
                Strafe(500, 0, 0.25);
                requestOpModeStop();
            }
            // Returns the current starting postion as an integer
            if (CSVisionProcessor.getIntPosition() == 3) {
                Outtake.setPosition(0);
                RobotHeadingChange = 0;
                sleep(200);
                Strafe(1000, 340, 0.25);
                Strafe(1000, 340, 0.25);
                LiftPositionUP(-50);
                sleep(500);
                Strafe(1750, 160, 0.25);
                sleep(500);
                Strafe(1700, 277, 0.3);
                sleep(500);
                RobotHeadingChange = -90;
                Strafe(1400, 0, 0.2);
                sleep(500);
                Strafe(700, 90, 0.25);
                LiftPositionUP(-400);
                sleep(500);
                Strafe(300, 0, 0.15);
                Outtake.setPosition(1);
                sleep(500);
                Strafe(300, 180, 0.25);
                sleep(500);
                Strafe(2200, 90, 0.25);
                sleep(500);
                Strafe(1100, 0, 0.25);
                requestOpModeStop();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Strafe(int Dist, int Deg, double Speed) {
        double EncoderDistAverage;
        int EncoderDistAverage_tare_;

        Dist = Dist * 1.1;
        M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EncoderDistA = (M1.getCurrentPosition() + M2.getCurrentPosition()) / 2;
        EncoderDistB = (R2a.getCurrentPosition() + R2b.getCurrentPosition()) / 2;
        EncoderDistAverage_tare_ = (EncoderDistA + EncoderDistB) / 2;
        EncoderDistAverage = (EncoderDistA + EncoderDistB) - EncoderDistAverage_tare_;
        while (!(Dist <= EncoderDistAverage)) {
            telemetry.addData("DecimalTravelled", (Dist - (Dist - EncoderDistAverage)) / Dist);
            EncoderDistAverage = (EncoderDistA + EncoderDistB) - EncoderDistAverage_tare_;
            SpeedOut = (Speed - Speed * ((Dist - (Dist - EncoderDistAverage)) / Dist)) * 4.2 * Speed + 0.05;
            telemetry.addData("SpeedOut", SpeedOut);
            DesiredAngle = Deg;
            KinematicsEmulator();
            telemetry.update();
            ModuleA();
        }
    }

    /**
     * Describe this function...
     */
    private void KinematicsEmulator() {
        float Yaw360;

        Yaw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        if (Yaw < 0) {
            Yaw360 = Yaw + 360;
        } else {
            Yaw360 = Yaw;
        }
        telemetry.addData("Yaw", Yaw);
        telemetry.addData("Yaw360", Yaw360);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void RotationModify() {
        int RotationDiffrence;
        int RotInvert;
        int RotInvertException;
        int VelocityAuthority;

        if (5 + RobotHeadingChange <= Yaw || -5 + RobotHeadingChange >= Yaw) {
            RightStickX = (Yaw - RobotHeadingChange) / 200;
        } else {
            RightStickX = 0;
        }
        if (DesiredAngle >= 0 && DesiredAngle <= 90) {
            RotationDiffrence = DesiredAngle / 90;
            RotInvert = 1;
            RotInvertException = 1;
        } else if (DesiredAngle > 90 && DesiredAngle <= 180) {
            RotationDiffrence = 1 - (DesiredAngle - 90) / 90;
            RotInvert = -1;
            RotInvertException = 1;
        } else if (DesiredAngle > 180 && DesiredAngle <= 270) {
            RotationDiffrence = (DesiredAngle - 180) / 90;
            RotInvert = -1;
            RotInvertException = 1;
        } else if (DesiredAngle > 270 && DesiredAngle <= 359) {
            RotationDiffrence = 1 - (DesiredAngle - 270) / 90;
            RotInvert = 1;
            RotInvertException = -1;
        } else {
            RotationDiffrence = 1;
            RotInvertException = 1;
        }
        RotationAuthority = RotationDiffrence * -1 * RotInvert * RotInvertException;
        VelocityAuthority = (1 - RotationDiffrence) * RotInvert;
        telemetry.addData("Rotation Authority", RotationAuthority);
        telemetry.addData("Velocity Authority", VelocityAuthority);
        telemetry.addData("FOCAngle", DesiredAngle);
        RotationModify2 = RightStickX * VelocityAuthority;
    }

    /**
     * Describe this function...
     */
    private void LiftAndDiffSetup() {
        Diff2.setDirection(Servo.Direction.REVERSE);
        Outtake.setPosition(1);
        Plane.setPosition(0.5);
        Diff1Rest = 0.12;
        Diff2Rest = 0.09;
        Diff2.setPosition(Diff2Rest);
        Diff1.setPosition(Diff1Rest);
        IntakeL.setDirection(CRServo.Direction.REVERSE);
        Lift1.setDirection(DcMotor.Direction.REVERSE);
        Lift2.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Describe this function...
     */
    private void LiftAndDiffLoop() {
        boolean IsRightTriggerPressed;
        double RightTrigger;
        boolean IsLeftTriggerPressed;

        telemetry.addData("Lift1Pos", Lift1.getCurrentPosition());
        if (RightTrigger > 0.1) {
            IsRightTriggerPressed = true;
        } else {
            IsRightTriggerPressed = false;
        }
        if (LeftTrigger > 0.1) {
            IsLeftTriggerPressed = true;
        } else {
            IsLeftTriggerPressed = false;
        }
        if (Lift1.getCurrentPosition() > -400) {
            if (IsRightTriggerPressed == true) {
                Lift1.setPower(RightTrigger * 0.2);
                Lift2.setPower(RightTrigger * 0.2);
            } else {
                Lift1.setPower(-1 * LeftTrigger * 0.4);
                Lift2.setPower(-1 * LeftTrigger * 0.4);
            }
        } else {
            if (IsRightTriggerPressed == true) {
                Lift1.setPower(RightTrigger);
                Lift2.setPower(RightTrigger);
            } else {
                Lift1.setPower(-1 * LeftTrigger);
                Lift2.setPower(-1 * LeftTrigger);
            }
        }
        if (Lift1.getCurrentPosition() > -70) {
            Diff2.setPosition(Diff2Rest + 0.01);
            Diff1.setPosition(Diff1Rest + 0.01);
        } else if (Lift1.getCurrentPosition() < -70 && Lift1.getCurrentPosition() > -350 || IsRightTriggerPressed == true) {
            Diff2.setPosition(Diff2Rest - 0.04);
            Diff1.setPosition(Diff1Rest - 0.04);
        } else {
            if (gamepad2.dpad_right) {
                Diff2.setPosition(Diff2Rest + 0.14);
                Diff1.setPosition(Diff1Rest + 0.43);
            } else {
                Diff2.setPosition(Diff2Rest + -0.08);
                Diff1.setPosition(Diff2Rest + 0.65);
            }
        }
        if (gamepad2.back) {
            Plane.setPosition(0.2);
        } else {
            Plane.setPosition(0.5);
        }
        if (gamepad2.a) {
            Outtake.setPosition(0);
        } else if (gamepad2.b) {
            Outtake.setPosition(1);
        }
        if (gamepad2.left_bumper) {
            IntakeL.setPower(1);
            IntakeR.setPower(1);
        } else if (gamepad2.right_bumper) {
            IntakeL.setPower(-1);
            IntakeR.setPower(-1);
        } else {
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void LiftPositionUP(int TickHeight) {
        while (!(TickHeight > Lift1.getCurrentPosition())) {
            LeftTrigger = 1;
            KinematicsEmulator();
            ModuleA();
        }
        LeftTrigger = 0;
        KinematicsEmulator();
        ModuleA();
    }

    /**
     * Describe this function...
     */
    private void IMU2() {
        BNO055IMU.Parameters IMU_Parameters;

        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);
    }

    /**
     * Describe this function...
     */
    private void ModuleA() {
        double velocity;
        double DesiredAngleR;
        double DesiredAngleR2;
        double DesiredAngleL;
        double DesiredAngleL2;
        double ModuleAngle0_360;
        double ModuleAngle0_3602;
        double ErrorR2;
        double ErrorL2;
        double ErrorR;
        double ErrorL;
        double Error2;
        double ErrorCorrection;
        double Error22;
        double ErrorCorrection2;

        LiftAndDiffLoop();
        RotationModify();
        DesiredAngleR = DesiredAngle + RightStickX * RotationAuthority * 180;
        DesiredAngleR2 = DesiredAngle - RightStickX * RotationAuthority * 180;
        DesiredAngleL = 360 - DesiredAngleR;
        DesiredAngleL2 = 360 - DesiredAngleR2;
        ModuleAngle0_360 = (EncoderA.getCurrentPosition() / 8192 - Math.floor(EncoderA.getCurrentPosition() / 8192)) * 360;
        ModuleAngle0_3602 = (EncoderB.getCurrentPosition() / 8192 - Math.floor(EncoderB.getCurrentPosition() / 8192)) * 360;
        if (DesiredAngleR2 > ModuleAngle0_3602) {
            ErrorR2 = DesiredAngleR2 - ModuleAngle0_3602;
        } else {
            ErrorR2 = 360 - (ModuleAngle0_3602 - DesiredAngleR2);
        }
        ErrorL2 = -(360 - ErrorR2);
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
        if (Math.abs(ErrorR2) < ErrorBand || Math.abs(ErrorL2) < ErrorBand) {
            Error22 = 0;
        } else if (Math.abs(ErrorR2) < Math.abs(ErrorL2)) {
            Error22 = ErrorR2 / 360;
        } else {
            Error22 = ErrorL2 / 360;
        }
        ErrorCorrection2 = Error22 * C1 + C2;
        velocity = SpeedOut;
        M1.setPower(velocity * 1.1 + ErrorCorrection + RotationModify2);
        M2.setPower((velocity * 1.1 - ErrorCorrection) + RotationModify2);
        R2a.setPower((velocity * 1 + ErrorCorrection2) - RotationModify2);
        R2b.setPower((velocity * 1 - ErrorCorrection2) - RotationModify2);
        EncoderDistA = (M1.getCurrentPosition() + M2.getCurrentPosition()) / 2;
        EncoderDistB = (R2a.getCurrentPosition() + R2b.getCurrentPosition()) / 2;
    }
}
