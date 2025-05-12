package teamcode.subsystems;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

public class CRServoArm extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "ServoArm";
        public static final boolean NEED_ZERO_CAL               = true;

        public static final FtcMotorActuator.MotorType MOTOR_TYPE= FtcMotorActuator.MotorType.CRServo;
        public static final String PRIMARY_CRSERVO_NAME         = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_CRSERVO_INVERTED    = false;
        public static final String FOLLOWER_CRSERVO_NAME        = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_CRSERVO_INVERTED   = true;

        public static final String EXTERNAL_ENCODER_NAME        = SUBSYSTEM_NAME + ".encoder";
        public static final boolean EXTERNAL_ENCODER_INVERTED   = false;

        public static final double DEG_PER_COUNT                = 0; //360
        public static final double ZERO_OFFSET                  = 0.0; //TBD
        public static final double POS_OFFSET                   = 0; //0
        public static final double POWER_LIMIT                  = 0.8;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 180.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double[] posPresets                 = {MIN_POS, 60.0, 90.0, 135.0, 180.0};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
                new TrcPidController.PidCoefficients(0.018, 0.1, 0.001, 0.0, 2.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.158;
    }   //class Params

    private final FtcDashboard dashboard;
    private final TrcMotor servoArm;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CRServoArm()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(Params.PRIMARY_CRSERVO_NAME, Params.MOTOR_TYPE, Params.PRIMARY_CRSERVO_INVERTED)
                .setFollowerMotor(Params.FOLLOWER_CRSERVO_NAME, Params.MOTOR_TYPE, Params.FOLLOWER_CRSERVO_INVERTED)
                .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET)
                .setExternalEncoder(Params.EXTERNAL_ENCODER_NAME, Params.EXTERNAL_ENCODER_INVERTED)
                .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        servoArm = new FtcMotorActuator(armParams).getMotor();
        servoArm.setPositionPidParameters(
                Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        servoArm.setPositionPidPowerComp(this::getGravityComp);
        servoArm.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }

    public TrcMotor getMotor()
    {
        return servoArm;
    }

    private double getGravityComp(double currPower)
    {
        return Params.GRAVITY_COMP_MAX_POWER * Math.cos(Math.toRadians(servoArm.getPosition()));
    }

    //
    // Implements TrcSubsystem abstract methods.
    //
    @Override
    public void cancel()
    {
        servoArm.cancel();
    }

    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // No zero calibration needed, when using absolute encoders.
    }

    @Override
    public void resetState()
    {
        servoArm.setPosition(Params.TURTLE_DELAY, Params.TURTLE_POS, true, Params.POWER_LIMIT);
    }   //resetState

    @Override
    public int updateStatus(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f",
                Params.SUBSYSTEM_NAME, servoArm.getPower(), servoArm.getCurrent(), servoArm.getPosition(), servoArm.getPidTarget());
        return lineNum;
    }   //updateStatus

}   //class Arm