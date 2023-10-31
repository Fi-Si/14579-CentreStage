package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

public class swerveKinematics {

    Translation2d module1Location = new Translation2d(0, 0.2);
    Translation2d module2Location =  new Translation2d(0, 0.2);

    SwerveDriveKinematics robotKinematics = new SwerveDriveKinematics(module1Location, module2Location);
    double[] moduleData;

    public double[] drive(double joystick1x, double  joystick1y, double joystick2x) {
        ChassisSpeeds speeds = new ChassisSpeeds(joystick1y, joystick1x, joystick2x);
        SwerveModuleState[] moduleStates = robotKinematics.toSwerveModuleStates(speeds);

        SwerveModuleState module1 = moduleStates[0];
       SwerveModuleState module2 = moduleStates[1];
        moduleData = new double[]{module1.speedMetersPerSecond, module1.angle.getDegrees(), module2.speedMetersPerSecond, module2.angle.getDegrees()};
        return moduleData;
    }
}

