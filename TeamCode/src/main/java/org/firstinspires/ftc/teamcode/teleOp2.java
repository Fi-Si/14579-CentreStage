package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class teleOp2  extends Robot{

    int counter;
    @Override
    public void init() {
       // Module.hardwareMap();
        hardwareInit();

    }

    @Override
    public void loop() {
        counter +=1;
        Kinematics.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        Module.driveToModule(Kinematics.moduleData[0], (int) Kinematics.moduleData[1]);
        telemetry.addData(">", counter);
        telemetry.update();
        module1a.setPower(100);
    }
}
