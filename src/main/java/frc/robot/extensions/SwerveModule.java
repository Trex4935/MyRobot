package frc.robot.extensions;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class SwerveModule {
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;
    

    public SwerveModule( WPI_TalonFX dMotor, WPI_TalonFX tMotor ) {

        driveMotor = dMotor;
        turnMotor = tMotor;

    }

    /**
     * This methods gives back turn encoder position.
     * @param selectedEncoder
     * @return value of selected encoder
     */
    public double getDriveEncoderTicks( ){
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * This methods gives back drive encoder position.
     * @param selectedEncoder
     * @return value of selected encoder
     */
    public double getTurnEncoderTicks( ){
      return turnMotor.getSelectedSensorPosition();
    }
    /**
     * Calculates angles
     * @return angle
     */
    public double getAngle(){
        return ticksToAngle(getTurnEncoderTicks(),Constants.turnMotorGearRatio);
    };
    /**
     *  Calculate distance
     * @return distance
     */
    public double getDistance(){
        return ticksToPosition(getDriveEncoderTicks( ), Constants.wheelDiameter , Constants.driveMotorGearRatio);
    };

    //TO DO SATURDAY;
    public double getSpeed(){
        return 10;
    };

    // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
    // travel distance. gear ratio A:1 means, 1/A. //Assuming all motors have same encoder per ticks.
    public double ticksToAngle(double ticks, double gearRatio) {
      double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
      double nbTurnWheel = nbTurnMotor * gearRatio;
      double angleOfWheel = nbTurnWheel * 360;
      return angleOfWheel;

    }

    // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
    // travel distance. gear ratio A:1 means, 1/A.
    public double ticksToPosition(double ticks, double wheelDiameter, double gearRatio) {
      double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
      double nbTurnWheel = nbTurnMotor * gearRatio;
      double distanceTravel = nbTurnWheel * Math.PI * wheelDiameter;
      return distanceTravel;
    }

    // Gives back the distance of the drive motor and the angle of the turn motor, as a swerve Module state Object 
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getSpeed(),new Rotation2d(getAngle()/(2*Math.PI)));
    };
}
