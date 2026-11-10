package org.firstinspires.ftc.teamcode;

public class TeleopDisplay {
    public static String convert(String Wheels, String Fl, String FR, String BL, String BR) {
            if ((Wheels).equals("^^^^") || (Wheels).equals("vvvv")) {
                if ((Wheels).equals("^^^^")) {
                    /* forward */
                    return "^^^^";
                } else {
                    /* Backward */
                    return "vvvv";
                }
            } else {
                if ((Wheels).equals("o^^o") || (Wheels).equals("voov")) {
                    if ((Wheels).equals("v^^v")) {
                        /* Left */
                        return "v^^v";
                    } else if ((Wheels).equals("o^^o")) {
                        /* Up left */
                        return "o^^o";
                    } else if ((Wheels).equals("voov")) {
                        /* down left */
                        return "voov";
                    }
                } else if ((Wheels).equals("^oo^") || (Wheels).equals("ovvo")) {
                    if ((Wheels).equals("^vv^")) {
                        /* Right */
                        return "^vv^";
                    } else if ((Wheels).equals("^oo^")) {
                        /* Up Right */
                        return "^oo^";
                    } else if ((Wheels).equals("ovvo")) {
                        /* down Right */
                        return "ovvo";
                    }
                }
            }
            return "N/A";
    }
}