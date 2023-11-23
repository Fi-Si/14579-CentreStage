package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CenterstageTeleOpMechOnly (Blocks to Java)")

public class CenterstageTeleOpMechOnly extends LinearOpMode {

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

    double Diff1Rest;
    double Diff2Rest;

    /**
     * Describe this function...
     */
    private void Climb() {
        if (gamepad2.dpad_down) {
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
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int MaxVelocity;
        int C1;
        int C2;
        int ErrorBand;

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

        // Put initialization blocks here.
        waitForStart();
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
                LiftAndDiffLoop();
                Climb();
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
}