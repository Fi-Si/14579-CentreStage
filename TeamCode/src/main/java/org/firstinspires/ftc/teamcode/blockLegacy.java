package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
@Disabled
public class blockLegacy extends LinearOpMode {

    private BNO055IMU imu;
    private DcMotor EncoderA;
    private DcMotor EncoderB;
    private DcMotor M1;
    private DcMotor R2a;
    private Servo Diff2;
    private Servo Outtake;
    private Servo Plane;
    private Servo Diff1;
    private CRServo IntakeL;
    private DcMotor Lift1;
    private DcMotor Lift2;
    private CRServo IntakeR;
    private DcMotor M2;
    private DcMotor R2b;

    double LeftStickY;
    int RotationModify2;
    double RightStickX;
    double Diff1Rest;
    double ModuleAngle0_360;
    double Diff2Rest;
    int DesiredAngle;
    double SpeedOut;
    int MaxVelocity;
    int C1;
    int C2;
    int ErrorBand;

    /**
     * Describe this function...
     */
    private void KinematicsEmulator() {
        double LeftStickX;
        float Yaw;
        double StickAngle;

        LeftStickX = gamepad1.left_stick_x;
        LeftStickY = gamepad1.left_stick_y;
        RightStickX = gamepad1.right_stick_x;
        if (0 < Math.abs(gamepad2.left_stick_x) || 0 < Math.abs(gamepad2.left_stick_y) || 0 < Math.abs(gamepad2.right_stick_x)) {
            LeftStickX = gamepad2.left_stick_x * 0.5;
            LeftStickY = gamepad2.left_stick_y * 0.5;
            RightStickX = gamepad2.right_stick_x * 0.5;
        } else {
            LeftStickX = gamepad1.left_stick_x;
            LeftStickY = gamepad1.left_stick_y;
            RightStickX = gamepad1.right_stick_x;
        }
        Yaw = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle;
        if (LeftStickX * -1 == 0 && LeftStickY == 0) {
            DesiredAngle = 0;
        } else {
            StickAngle = Math.atan((LeftStickX * -1) / (LeftStickY * -1)) / Math.PI * 180;
            if (LeftStickY * -1 <= 0) {
                DesiredAngle = (int) (StickAngle + 180);
            } else if (LeftStickX * -1 < 0) {
                DesiredAngle = (int) (StickAngle + 360);
            } else {
                DesiredAngle = (int) StickAngle;
            }
        }
        SpeedOut = Math.sqrt(Math.pow(LeftStickX * -1, 2) + Math.pow(LeftStickY * -1, 2)) * MaxVelocity;
        telemetry.addData("Yaw", Yaw);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void Climb() {
        if ((gamepad2.dpad_right || gamepad2.dpad_left) && gamepad2.dpad_down) {
            EncoderA.setPower(-0.2);
            EncoderB.setPower(0.2);
        } else if ((gamepad2.dpad_right || gamepad2.dpad_left) && gamepad2.dpad_up) {
            EncoderA.setPower(0.2);
            EncoderB.setPower(-0.2);
        } else if (gamepad2.dpad_down) {
            EncoderA.setPower(-1);
            EncoderB.setPower(1);
        } else if (gamepad2.dpad_up) {
            EncoderA.setPower(1);
            EncoderB.setPower(-1);
        } else {
            EncoderA.setPower(0);
            EncoderB.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void RotationModify() {
        if (ModuleAngle0_360 <= 70 || ModuleAngle0_360 >= 290 || ModuleAngle0_360 <= 210 && ModuleAngle0_360 >= 150) {
            if (ModuleAngle0_360 < 270 && ModuleAngle0_360 > 90) {
                RotationModify2 = (int) ((RightStickX - LeftStickY * 0.05) * -0.78);
            } else {
                RotationModify2 = (int) ((RightStickX - LeftStickY * 0.05) * 0.78);
            }
        } else {
            RotationModify2 = 0;
        }
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        EncoderA = hardwareMap.get(DcMotor.class, "EncoderA");
        EncoderB = hardwareMap.get(DcMotor.class, "EncoderB");
        M1 = hardwareMap.get(DcMotor.class, "M1");
        R2a = hardwareMap.get(DcMotor.class, "R2a");
        Diff2 = hardwareMap.get(Servo.class, "Diff2");
        Outtake = hardwareMap.get(Servo.class, "Outtake");
        Plane = hardwareMap.get(Servo.class, "Plane");
        Diff1 = hardwareMap.get(Servo.class, "Diff1");
        IntakeL = hardwareMap.get(CRServo.class, "IntakeL");
        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        IntakeR = hardwareMap.get(CRServo.class, "IntakeR");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        R2b = hardwareMap.get(DcMotor.class, "R2b");

        // Put initialization blocks here.
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
            while (opModeIsActive()) {
                ModuleA();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void LiftAndDiffSetup() {
        Diff2.setDirection(Servo.Direction.REVERSE);
        Outtake.setPosition(0);
        Plane.setPosition(0.5);
        Diff1Rest = 0 + 0.2;
        Diff2Rest = 0.7 - 0.25;
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
        boolean IsLeftTriggerPressed;

        telemetry.addData("Lift1Pos", Lift1.getCurrentPosition());
        if (gamepad2.right_trigger > 0.1) {
            IsRightTriggerPressed = true;
        } else {
            IsRightTriggerPressed = false;
        }
        if (gamepad2.left_trigger > 0.1) {
            IsLeftTriggerPressed = true;
        } else {
            IsLeftTriggerPressed = false;
        }
        if (Lift1.getCurrentPosition() > -400) {
            if (IsRightTriggerPressed == true) {
                Lift1.setPower(gamepad2.right_trigger * 0.2);
                Lift2.setPower(gamepad2.right_trigger * 0.2);
            } else {
                Lift1.setPower(-1 * gamepad2.left_trigger * 0.4);
                Lift2.setPower(-1 * gamepad2.left_trigger * 0.4);
            }
        } else {
            if (IsRightTriggerPressed == true) {
                Lift1.setPower(gamepad2.right_trigger);
                Lift2.setPower(gamepad2.right_trigger);
            } else {
                Lift1.setPower(-1 * gamepad2.left_trigger);
                Lift2.setPower(-1 * gamepad2.left_trigger);
            }
        }
        if (Lift1.getCurrentPosition() > -70) {
            Diff2.setPosition(Diff2Rest);
            Diff1.setPosition(Diff1Rest);
        } else if (Lift1.getCurrentPosition() < -70 && Lift1.getCurrentPosition() > -350 || IsRightTriggerPressed == true) {
            Diff2.setPosition(Diff2Rest - 0.05);
            Diff1.setPosition(Diff1Rest - 0.05);
        } else {
            if (gamepad2.dpad_right) {
                Diff2.setPosition(Diff2Rest - 0.2);
                Diff1.setPosition(Diff1Rest - 0.41);
            } else {
                Diff2.setPosition(Diff2Rest - (0.7 - 0.15));
                Diff1.setPosition(Diff2Rest - (0.1 - 0.5));
            }
        }
        if (gamepad2.back) {
            Plane.setPosition(0.2);
        } else {
            Plane.setPosition(0.5);
        }
        while (gamepad2.a) {
            Outtake.setPosition(0.25);
        }
        while (gamepad2.b) {
            Outtake.setPosition(0.6);
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
        double DesiredAngleR;
        int DesiredAngleR2;
        double DesiredAngleL;
        int DesiredAngleL2;
        double ModuleAngle0_3602;
        double ErrorR2;
        double ErrorL2;
        double ErrorR;
        double ErrorL;
        double Error2;
        double ErrorCorrection;
        double Error22;
        double ErrorCorrection2;
        double velocity;

        while (opModeIsActive()) {
            KinematicsEmulator();
            LiftAndDiffLoop();
            Climb();
            DesiredAngleR = DesiredAngle;
            DesiredAngleR2 = DesiredAngle;
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
            RotationModify();
            M1.setPower(velocity + ErrorCorrection + RotationModify2);
            M2.setPower((velocity - ErrorCorrection) + RotationModify2);
            R2a.setPower((velocity + ErrorCorrection2) - RotationModify2);
            R2b.setPower((velocity - ErrorCorrection2) - RotationModify2);
        }
    }
}