package teamcode.subsystems;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcServoActuator;
import ftclib.sensor.FtcAnalogEncoder;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcAbsoluteEncoder;
import trclib.subsystem.TrcSubsystem;

public class ServoArm extends TrcSubsystem
{
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "ServoArm";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String PRIMARY_SERVO_NAME           = Params.SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;
        public static final String FOLLOWER_SERVO_NAME          = Params.SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = false;

        public static final boolean useExternalEncoder          = true;
        public static final String EXTERNAL_ENCODER_NAME        = SUBSYSTEM_NAME + ".encoder";
        public static final boolean EXTERNAL_ENCODER_INVERTED   = false;
        public static final double DEG_PER_COUNT                = 0; //360
        public static final double ZERO_OFFSET                  = 0.0; //TBD
        public static final double POS_OFFSET                   = 0; //0

        public static final double POS_0_DEGREES                = 0.1;
        public static final double POS_90_DEGREES               = 0.8;
    }   //class Params

    private final FtcDashboard dashboard;
    public final TrcServo servoArm;
    private final FtcAnalogEncoder externalEncoder;

    public ServoArm() {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FtcDashboard.getInstance();
        FtcServoActuator.Params armParams = new FtcServoActuator.Params()
                .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
                .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED);
        servoArm = new FtcServoActuator(armParams).getServo();

        if (Params.useExternalEncoder)
        {
        externalEncoder = new FtcAnalogEncoder(Params.EXTERNAL_ENCODER_NAME);
        externalEncoder.setInverted(Params.EXTERNAL_ENCODER_INVERTED);
        externalEncoder.setScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET);
        }
        else externalEncoder = null;
    }

    public boolean is90Degrees()
    {
        return servoArm.getPosition() == Params.POS_90_DEGREES;
    }

    public void setArm90Degrees()
    {
        servoArm.setPosition(null, 0.0, Params.POS_90_DEGREES, null, 0.0);
    }

    public void setArm0Degrees()
    {
        servoArm.setPosition(null, 0.0, Params.POS_0_DEGREES, null, 0.0);
    }

    @Override
    public void cancel()
    {
        servoArm.cancel();
    }   //cancel

    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // No zero calibration needed.
    }   //zeroCalibrate

    @Override
    public void resetState()
    {
        servoArm.setPosition(Params.POS_90_DEGREES);
    }   //resetState

    @Override
    public int updateStatus(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum++, "%s: pos=%.3f, 90Degrees=%s", Params.SUBSYSTEM_NAME, externalEncoder != null ? externalEncoder.getScaledPosition() : servoArm.getPosition(), is90Degrees());
        return lineNum;
    }   //updateStatus

}   //class ServoExtender