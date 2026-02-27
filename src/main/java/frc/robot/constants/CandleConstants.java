package frc.robot.constants;

public class CandleConstants {
    public enum CandleState {
        IDLE,
        DISABLED,
        /*
         * ELEVATOR_ERROR,
         * CORAL_LASER_EXIT_ERROR,
         * CORAL_LASER_ENTRY_ERROR,
         * FINE_GRAIN_DRIVE,
         * CORAL_ENTERING,
         * HAS_CORAL,
         * ELEVATOR_STOW,
         * ELEVATOR_L2,
         * ELEVATOR_L3,
         * ELEVATOR_L4,
         * SCORING,
         * AUTO_ALIGN
         */
    }

    public static final int CAN_ID = 31;
    public static final int LED_LENGTH = 1000;
    public static final String CAN_BUS_NAME = "bye";
}
