package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Cole Tucker This enum encompasses all user selectable autonomous
 * commands.
 * <p>
 * It has two parameters, a visible name and an associated ID number
 * from 1 to n.
 */


public class AutoSelector {

    /**
     * Method to get an array of names for all selections
     *
     * @return string array of enum names
     */
    public static String[] buildArray() {
        String[] out = new String[UserSelection.values().length];
        for (int i = 0; i < UserSelection.values().length; i++) {
            out[i] = UserSelection.values()[i].toString();
        }
        return out;
    }

    /**
     * Gets the enum object that matches the fed string.
     *
     * @param name checked against all possible enum name parameters
     * @return The internally used enum for auto calculations
     */
    private static UserSelection getSelFromStr(String name) {
        for (UserSelection sel : UserSelection.values()) {
            if (sel.toString().equalsIgnoreCase(name)) {
                return sel;
            }
        }
        // return the last possible enum by default
        return UserSelection.values()[UserSelection.values().length-1];
    }

    private static int getFieldPos(String GameData) {
        GameData = GameData.toLowerCase(); // ensure all data in known form
        //Switch first then Scale
        char c0 = GameData.charAt(0);
        char c1 = GameData.charAt(1);
        //System.out.println("Game data " + c0 + " " + c1);
        if (c0 == 'l' && c1 == 'l') {
            return 1; //LL
        } else if (c0 == 'l' && c1 == 'r') {
            return 2; //LR
        } else if (c0 == 'r' && c1 == 'l') {
            return 3; //RL
        } else {
            return 4; //RR
        }
    }

    /**
     * This method determines the Auto mode based on the fed game data and the
     * dashboard data.
     *
     * @param GameData  - a string containing the field positions of the team objectives.
     * @param selection - a string with the name of the selected autonomous.
     * @return the proper auto command to run. It should include all movements in
     * one command
     */
    public static void autoSelect(String GameData, String selection) {
        int usrAuto = getSelFromStr(selection).getNum();
        int fieldPos = getFieldPos(GameData);
        SmartDashboard.putNumber("Final Auto Choice", (usrAuto * 10 + fieldPos));
        switch (usrAuto * 10 + fieldPos) {

            /*case 11:
            case 12:
            case 13:
            case 14:

            case 21:
            case 22:
            case 23:
            case 24:

            case 31:
            case 32:
            case 33:
            case 34:

            case 41:
            case 42:
            case 43:
            case 44:

            case 51:
            case 52:
            case 53:
            case 54:

            case 61:
            case 62:
            case 63:
            case 64:

            case 71:
            case 72:
            case 73:
            case 74:

            case 81:
            case 82:
            case 83:
            case 84:

            case 91:
            case 92:
            case 93:
            case 94:

            case 101:
            case 102:
            case 103:
            case 104:

            case 111:
            case 112:
            case 113:
            case 114: */

            default:
        }
    }
    enum UserSelection {

        Auto1("", 1),
        Auto2("", 2),
        Auto3("", 3),
        Auto4("", 4),
        Auto5("", 5),
        Auto6("",6),
        Auto7("", 7),
        Auto8("",8),
        Auto9("", 9),
        Auto10("", 10),
        Auto11("", 11),
        Auto20("", 20);

        private String name;
        private int num;

        UserSelection(String name, int num) {
            this.name = name;
            this.num = num;
        }

        @Override
        public String toString() {
            return name;
        }

        public int getNum() {
            return num;
        }

    }
}