package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MechanumDrivebase")
public class MechanumDrivebase extends LinearOpMode {

    public DcMotor RFMotor;
    public DcMotor LBMotor;
    public DcMotor LFMotor;
    public DcMotor RBMotor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float y;
        double x;
        float rx;
        double denominator;

        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        // Reverse the right side motors.  This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards, reverse the left side instead.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            // Remember, Y stick value is reversed
            y = -gamepad1.left_stick_y;
            // Factor to counteract imperfect strafing
            x = gamepad1.right_stick_x * -1.1;
            rx = gamepad1.left_stick_x;
            // Denominator is the largest motor power (absolute value) or 1.
            // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            // Make sure your ID's match your configuration
            LFMotor.setPower((y + x + rx) / denominator);
            LBMotor.setPower(((y - x) + rx) / denominator);
            RFMotor.setPower(((y - x) - rx) / denominator);
            RBMotor.setPower(((y + x) - rx) / denominator);
        }
    }
}