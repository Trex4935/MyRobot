package frc.robot.extensions;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class SwerveModule {
    //Motors
    WPI_TalonFX driveMotor;
    WPI_TalonFX turnMotor;

    //Encoders (Replace Encoder with actual encoders used)
    Encoder driveEncoder;
    Encoder turnEncoder;

    //Absolute Encoder
    AnalogInput absoluteEncoder;
    boolean absoluteEncoderReversed;
    double absoluteEncoderOffsetRad;

    //PID Controller (Change to Xbox controller later?)
    PIDController turningPIDController;


    public SwerveModule(WPI_TalonFX dMotor, WPI_TalonFX tMotor, boolean driveMotorReversed, boolean turnMotorReversed, AnalogInput absoluteEncoder, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        driveMotor = dMotor;
        turnMotor = tMotor;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(Constants.absoluteEncoderID);

        //Placeholders
        driveMotor = new WPI_TalonFX(Constants.dtbackleftmotorID);
        turnMotor = new WPI_TalonFX(Constants.turnbackleftmotorID);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        //
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        //Need to add/change .set---ConversionFactor
        driveEncoder.setPositionConversionFactor();
        driveEncoder.setVelocityConversionFactor();
        turnEncoder.setPositionConversionFactor();
        turnEncoder.setVelocityConversionFactor();

        //PID Controller
        turningPIDController = new PIDController(Constants.kPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);


    }


    //Next four methods will probably change
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return driveEncoder.getVelocity();
    }


    /* Gives absolute encoder radians based on voltage and whether or not it is reversed.
     * Need to replace getVoltage5V */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage()/ controller.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * This methods gives back turn encoder position.
     * @param selectedEncoder
     * @return value of selected encoder
     */
    //Replace driveMotor w/ driveEncoder and turnMotor w/ turnEncoder?
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
