package frc.robot.extensions;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

public class SwerveModule {
    //Motors
    WPI_TalonSRX driveMotor;
    CANSparkMax turnMotor;

    //Encoders (Replace Encoder with actual encoders used)
    RelativeEncoder turnEncoder;

    //Absolute Encoder
    AnalogInput absoluteEncoder;
    boolean absoluteEncoderReversed;
    double absoluteEncoderOffsetRad;

    //PID Controller
    SparkMaxPIDController turningPIDController;

// Creates a swerve module from the motors and the absolute encoder
    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        //Motors
        //driveMotor = dMotor;
        //turnMotor = tMotor;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        //Creates an absolute encoder object
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        //Creates drive and turn motor objects
        driveMotor = new WPI_TalonSRX(driveMotorID);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        //Creates turn encoder object
        turnEncoder = turnMotor.getEncoder();

        turnEncoder.setPositionConversionFactor(Constants.turnEncoderRadians);
        turnEncoder.setVelocityConversionFactor(Constants.turnEncoderRPMRadPerSec);

        //Creates PID controller object
        turningPIDController = turnMotor.getPIDController();
        turningPIDController.setP(Constants.kPTurning);
        turningPIDController.setI(0);
        turningPIDController.setD(0);
        //Replacement?
        //turningPIDController.enableContinuousInput(-Math.PI, Math.PI);


    }


    //Returns the turn position

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    //Returns the turn velocity

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }


    // Gives absolute encoder radians based on voltage and whether or not it is reversed.
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage()/ RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetTurnEncoder() {
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * This methods gives back drive encoder position.
     * @param selectedEncoder
     * @return value of selected encoder
     */

      public double getDriveEncoderTicks( ){
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * This method gives back turn encoder position.
     * @param selectedEncoder
     * @return value of selected encoder
     */
    public double getTurnEncoderTicks( ){
        //Change to kAbsolute?
      return turnMotor.getAnalog(Mode.kRelative).getPosition();
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
        return ticksToPosition(getDriveEncoderTicks(), Constants.wheelDiameter , Constants.driveMotorGearRatio);
    };


    public double getSpeed(double ticksVelocity, double wheelDiameter) {
        double radius = wheelDiameter / 2;
        double turnsPerSec = ticksVelocity * 1000;
        double speedPerSec = turnsPerSec * ((2 * Math.PI * radius) / Constants.encoderTicksPerTurn);
        return speedPerSec;
    }

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
        return new SwerveModuleState(getSpeed(getDriveEncoderTicks(), Constants.wheelDiameter),new Rotation2d(getAngle()/(2*Math.PI)));
    };

    //
    public void setDesiredState(SwerveModuleState sModuleState) {
        if (Math.abs(sModuleState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        sModuleState = SwerveModuleState.optimize(sModuleState, getModuleState().angle);

        driveMotor.set(sModuleState.speedMetersPerSecond / Constants.dtMaxSpeed);
        //Replacement?
        //turnMotor.set(turningPIDController.calculate(getTurnPosition(), sModuleState.angle.getRadians()));
        System.out.println("Swerve[" + absoluteEncoder.getChannel() + "] state");
        System.out.println(sModuleState.toString());
        /*Used in video:
         * SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", sModuleState.toString());
         */
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
