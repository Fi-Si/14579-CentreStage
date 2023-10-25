package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class teleOp2  extends Robot{

    @Override
    public void init() {
       Module.hardwareMap();
        hardwareInit();
    }

    @Override
    public void loop() {

        Kinematics.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
      Module.driveToModule(Kinematics.moduleData[0], (int) Kinematics.moduleData[1]);
    }
}
