package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "CentreStageTeleOp (Main)")
public class CentreStageTeleOp extends LinearOpMode {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Diff1Rest;
        double Diff2Rest;
        float y;
        double x;
        float rx;
        double denominator;
        boolean IsRightTriggerPressed;

        DcMotor RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        DcMotor LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        Servo outtake = hardwareMap.get(Servo.class, "Outtake");
        Servo diff2 = hardwareMap.get(Servo.class, "Diff2");
        Servo diff1 = hardwareMap.get(Servo.class, "Diff1");
        CRServo leftIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        DcMotor lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        DcMotor LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        DcMotor RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        DcMotor lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        CRServo rightIntake = hardwareMap.get(CRServo.class, "RightIntake");

        // Reverse the right side motors.  This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards, reverse the left side instead.
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        outtake.setPosition(0);
        diff2.setDirection(Servo.Direction.REVERSE);
        Diff1Rest = 0 + 0.2;
        Diff2Rest = 0.7 - 0.25;
        diff2.setPosition(Diff2Rest);
        diff1.setPosition(Diff1Rest);
        leftIntake.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("MotorPos", lift1.getCurrentPosition());
            telemetry.update();
            // Remember, Y stick value is reversed
            y = -gamepad1.left_stick_y;
            // Factor to counteract imperfect strafing
            x = gamepad1.left_stick_x * -1.1;
            rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1.
            // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            // Make sure your ID's match your configuration
            LFMotor.setPower((y + x + rx) / denominator);
            LBMotor.setPower(((y - x) + rx) / denominator);
            RFMotor.setPower(((y - x) - rx) / denominator);
            RBMotor.setPower(((y + x) - rx) / denominator);
            IsRightTriggerPressed = gamepad1.right_trigger > 0.1;
            if (lift1.getCurrentPosition() > -400) {
                if (IsRightTriggerPressed) {
                    lift1.setPower(gamepad1.right_trigger * 0.2);
                    lift2.setPower(gamepad1.right_trigger * 0.2);
                } else {
                    lift1.setPower(-1 * gamepad1.left_trigger * 0.4);
                    lift2.setPower(-1 * gamepad1.left_trigger * 0.4);
                }
            } else {
                if (IsRightTriggerPressed) {
                    lift1.setPower(gamepad1.right_trigger);
                    lift2.setPower(gamepad1.right_trigger);
                } else {
                    lift1.setPower(-1 * gamepad1.left_trigger);
                    lift2.setPower(-1 * gamepad1.left_trigger);
                }
            }
            if (lift1.getCurrentPosition() > -140) {
                diff2.setPosition(Diff2Rest + 0);
                diff1.setPosition(Diff1Rest + 0);
            } else if (lift1.getCurrentPosition() < -140 && lift1.getCurrentPosition() > -350) {
                diff2.setPosition(Diff2Rest - 0.1);
                diff1.setPosition(Diff1Rest - 0.1);
            } else {
                if (gamepad1.dpad_right) {
                    diff2.setPosition(Diff2Rest + 0.2);
                    diff1.setPosition(Diff1Rest + 0.41);
                } else {
                    diff2.setPosition(Diff2Rest - (0.7 - 0.17));
                    diff1.setPosition(Diff2Rest + 0.1 + 0.3);
                }
            }
            while (gamepad1.a) {
                outtake.setPosition(0);
            }
            while (gamepad1.b) {
                outtake.setPosition(0.25);
            }
            if (gamepad1.left_bumper) {
                leftIntake.setPower(1);
                rightIntake.setPower(1);
            } else if (gamepad1.right_bumper) {
                leftIntake.setPower(-1);
                rightIntake.setPower(-1);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
        }
    }
}