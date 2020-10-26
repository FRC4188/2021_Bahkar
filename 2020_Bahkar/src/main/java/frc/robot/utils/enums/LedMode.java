package frc.robot.utils.enums;

/**
  * Enum to control LED mode.
  */
  public enum LedMode {
    DEFAULT(0), OFF(1), BLINK(2), ON(3);

    private final int value;
    LedMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  }