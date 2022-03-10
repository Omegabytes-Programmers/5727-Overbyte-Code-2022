public static class Browning {
    public static double getBatteryVoltage() {
        return PowerJNI.getVinVoltage();
    }

    public static boolean isBrownedOut() {
        if (HAL.getBrownedOut()) {
            System.out.println("the bot has shat itself")
         } else {
            System.out.println("the bot has not shat itself")
         }
    }
}