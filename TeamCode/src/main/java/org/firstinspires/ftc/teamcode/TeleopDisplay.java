package org.firstinspires.ftc.teamcode;

public class TeleopDisplay {
    public static String convert(String Wheels, double FL, double FR, double BL, double BR) {
        if ((Wheels).equals("^^^^") || (Wheels).equals("vvvv")) {
            if ((Wheels).equals("^^^^")) {
                return "Forward";  /* forward */
            } else {
                return "Backward"; /* Backward */
            }
        } else if (FL > FR) { /* Rightward Movement */
            if (BL > BR) {
                return "Rotating Clockwise";
            } else if ((FL + BR) > (FR + BL)) {
                if ((FL + BR > 0) && (FR + BL < 0)) {
                    return "Strafing Right";
                } else if ((FL + BR) > 0) {
                    return "Diagonal Up right";
                } else {
                    return "Diagonal Down right";
                }
            }
        } else if (FL < FR) { /* Leftward Movement */
            if (BL < BR) {
                return "Rotating CounterClockwise";
            } else if ((FL + BR) < (FR + BL)) {
                if ((FL + BR < 0) && (FR + BL > 0)) {
                    return "Strafing Left";
                } else if ((FR + BL) > 0) {
                    return "Diagonal Up Left";
                } else {
                    return "Diagonal Down Left";
                }
            }
        }
        return "N/A Any Movement";
    }
}
/* else if ( (FL > 0 && BR > 0) || (FR < 0 && BL < 0) ) {
        return "Strafing Right"; */