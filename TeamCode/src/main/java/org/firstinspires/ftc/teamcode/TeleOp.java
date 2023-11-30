package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    LiftAndDiff mechanisms;
    Climb climber;


    //deadband for joysticks
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);

    public boolean willResetIMU = true;

    public void init() {
        robot = new Robot(this, false);
        mechanisms = new LiftAndDiff(robot.Lift1, robot.Lift2, robot.IntakeL, robot.IntakeR, robot.Outtake, robot.Plane, robot.Diff1, robot.Diff2);
        climber = new Climb(robot.climb1, robot.climb2);
        mechanisms.LiftAndDiffSetup();
    }

    //allows driver to indicate that the IMU should not be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
    }
    public void start () {
        if (willResetIMU) robot.initIMU();
    }


    public void loop() {
        Vector2d joystick1A = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        Vector2d joystick2A = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick
        Vector2d joystick1B = new Vector2d(gamepad2.left_stick_x, -gamepad2.left_stick_y); //LEFT joystick
        Vector2d joystick2B = new Vector2d(gamepad2.right_stick_x, -gamepad2.right_stick_y); //RIGHT joystick
        if (-0.1 < gamepad2.left_stick_x && gamepad2.left_stick_x < 0.1 && -0.1 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.1 && -0.1 < gamepad2.right_stick_x && gamepad2.right_stick_x < 0.1 && -0.1 < gamepad2.right_stick_y && gamepad2.right_stick_y < 0.1) {
            robot.driveController.updateUsingJoysticks(checkDeadband(joystick1A), checkDeadband(joystick2A));
        }
        else {
            robot.driveController.updateUsingJoysticks(checkDeadband(joystick1B), checkDeadband(joystick2B));
        }
        mechanisms.LiftAndDiffLoop();
        climber.climb();

        //uncomment for live tuning of ROT_ADVANTAGE constant
        /*if (gamepad1.b) {
          robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
        robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
        }
        if (gamepad1.x) {
          robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
        }
      telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


        if(gamepad1.y) {
            robot.driveController.moduleRight.TICKS_PER_MODULE_REV += 5;
            robot.driveController.moduleLeft.TICKS_PER_MODULE_REV += 5;
        }
        if(gamepad1.a) {
            robot.driveController.moduleRight.TICKS_PER_MODULE_REV -= 5;
            robot.driveController.moduleLeft.TICKS_PER_MODULE_REV -= 5;
        }
        telemetry.addData("TICKS_PER_MODULE_REV: ", robot.driveController.moduleLeft.TICKS_PER_MODULE_REV);
        */

        //to confirm that joysticks are operating properly
        telemetry.addData("Joystick 1", joystick1A);
        telemetry.addData("Joystick 2", joystick2A);

        telemetry.update();
    }

    //returns zero vector if joystick is within deadband
    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return Vector2d.ZERO;
    }
}