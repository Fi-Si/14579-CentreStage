package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SwerveTest (Blocks to Java)")
class SwerveTest extends LinearOpMode {

    public DcMotor motorA;
    public DcMotor motorB;
    public DcMotor motorC;
    public DcMotor motorD;
    public DcMotor motorE;
    public DcMotor motorF;
    float aInput;
    float bInput;

    /**
     * Describe this function...
     */
    private void Run(float Swerve, float Rotate) {
        aInput= Swerve + Rotate;
        bInput= Swerve - Rotate;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotor.class, "A");
        motorB = hardwareMap.get(DcMotor.class, "B");
        motorC = hardwareMap.get(DcMotor.class, "C");
        motorD = hardwareMap.get(DcMotor.class, "D");
        motorE = hardwareMap.get(DcMotor.class, "E");
        motorF = hardwareMap.get(DcMotor.class, "F");
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Run(gamepad1.right_stick_x, gamepad1.right_stick_y);
                motorA.setDirection(DcMotorSimple.Direction.FORWARD);
                motorB.setDirection(DcMotorSimple.Direction.FORWARD);
                motorA.setPower(aInput);
                motorB.setPower(bInput);
                motorC.setPower(aInput);
                motorD.setPower(bInput);
                motorE.setPower(aInput);
                motorF.setPower(bInput);
                telemetry.update();
            }
        }
    }
}