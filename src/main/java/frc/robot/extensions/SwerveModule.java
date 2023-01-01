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
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

public class SwerveModule {
    //Motors
    WPI_TalonSRX driveMotor;
    CANSparkMax turnMotor;

    //Encoder
    RelativeEncoder turnEncoder;

    //Absolute Encoder
    AnalogInput absoluteEncoder;
    boolean absoluteEncoderReversed;
    double absoluteEncoderOffsetRad;

    //PID Controller
    SparkMaxPIDController turningPIDController;

/** Creates a swerve module from the motors and the absolute encoder. */
    public SwerveModule(int driveMotorID, 
    int turnMotorID, 
    boolean driveMotorReversed, 
    boolean turnMotorReversed, 
    int absoluteEncoderID, 
    double absoluteEncoderOffset, 
    boolean absoluteEncoderReversed) {


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
        turningPIDController.setIZone(0);
        turningPIDController.setD(0);
        turningPIDController.setFF(0);
        //Not sure what the reference values would be
        turningPIDController.setReference(0, ControlType.kVelocity, 0);
        turningPIDController.setReference(0, ControlType.kVelocity, 1);
        turningPIDController.setReference(0, ControlType.kVelocity, 2);
        turningPIDController.setReference(0, ControlType.kVelocity, 3);

        //May work as a replacement for turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        turningPIDController.setOutputRange(-Math.PI, Math.PI);

        turningPIDController.setFeedbackDevice(turnEncoder);

    }


    /** Returns the turn position. */

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /** Returns the turn velocity. */

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }


    /** Gives absolute encoder radians based on voltage and whether or not it is reversed. */
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage()/ RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /** Resets the turn encoder. */
    public void resetTurnEncoder() {
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * This method gives back drive encoder position.
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
     * Calculates angles.
     * @return angle
     */
    public double getAngle(){
        return ticksToAngle(getTurnEncoderTicks(), Constants.turnMotorGearRatio);
    };
    /**
     *  Calculate distance.
     * @return distance
     */
    public double getDistance(){
        return ticksToPosition(getDriveEncoderTicks(), Constants.wheelDiameter , Constants.driveMotorGearRatio);
    };

    /** Returns the speed. */
    public double getSpeed() {
        double radius = Constants.wheelDiameter / 2;
        double turnsPerSec = getDriveEncoderTicks() * 1000;
        double speedPerSec = turnsPerSec * ((2 * Math.PI * radius) / Constants.encoderTicksPerTurn);
        return speedPerSec;
    }

    /** Takes the rotation or internal ticks of Falcon Encoder and turn them to a
    travel distance. gear ratio A:1 means, 1/A. Assumes all motors have same encoder per ticks. */
    public double ticksToAngle(double ticks, double gearRatio) {
      double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
      double nbTurnWheel = nbTurnMotor * gearRatio;
      double angleOfWheel = nbTurnWheel * 360;
      return angleOfWheel;

    }

    /** Takes the rotation or internal ticks of Falcon Encoder and turn them to a 
    travel distance. gear ratio A:1 means, 1/A. */
    public double ticksToPosition(double ticks, double wheelDiameter, double gearRatio) {
      double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
      double nbTurnWheel = nbTurnMotor * gearRatio;
      double distanceTravel = nbTurnWheel * Math.PI * wheelDiameter;
      return distanceTravel;
    }

    /** Gives back the distance of the drive motor and the angle of the turn motor, as 
     a swerve module state object. */ 
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getSpeed(),new Rotation2d(getAngle()/(2*Math.PI)));
    };

    /** 
     * Stops the motors if the speed is negligble.
     * @param sModuleState
     */
    public void setDesiredState(SwerveModuleState sModuleState) {
        if (Math.abs(sModuleState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        sModuleState = SwerveModuleState.optimize(sModuleState, getModuleState().angle);

        driveMotor.set(sModuleState.speedMetersPerSecond / Constants.dtMaxSpeed);
        //May work as a replacement for turnMotor.set(turningPIDController.calculate(getTurnPosition(), sModuleState.angle.getRadians()));
        turnMotor.set(turningPIDController.getSmartMotionAllowedClosedLoopError(0));
        System.out.println("Swerve[" + absoluteEncoder.getChannel() + "] state");
        System.out.println(sModuleState.toString());
        /*Used in video:
         * SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", sModuleState.toString());
         */
    }

    /** Stops the motors. */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
