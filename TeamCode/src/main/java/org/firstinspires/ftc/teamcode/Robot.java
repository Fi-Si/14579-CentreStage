package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;
public abstract class Robot extends OpMode {


DcMotorEx module1a, module1b, module2a, module2b, lift1, lift2;

CRServo intake1, intake2;

Servo arm1, arm2, outtake;

public swerveKinematics Kinematics = null;
public module Module = null;

    public void hardwareInit() {
        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub: allHubs) {
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
                List<DcMotorEx> modules;

        //Module 1 Motors
        module1a = hardwareMap.get(DcMotorEx.class, "M1");
        module1b = hardwareMap.get(DcMotorEx.class, "M2");
        module1b.setDirection(DcMotorSimple.Direction.REVERSE);
        //Module 2 Motors
      //  module2a = hardwareMap.get(DcMotorEx.class, "module2a");
       // module2b = hardwareMap.get(DcMotorEx.class, "module2b");
        //Lift Motors
       // lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
       // lift2 = hardwareMap.get(DcMotorEx.class, "Lift2");

        modules = Arrays.asList(module1a, module1b/* , module2a, module2b */);
        for(DcMotorEx motor: modules) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        Kinematics = new swerveKinematics();
        Module = new module(module1a, module1b);

    }

    // Create a Main class
   /* public class Main {
            public Main() {

        }

        public static void main(String[] args) {
            Main myObj = new Main(); // Create an object of class Main (This will call the constructor)
            System.out.println(myObj.x); // Print the value of x
        }
    */ }

