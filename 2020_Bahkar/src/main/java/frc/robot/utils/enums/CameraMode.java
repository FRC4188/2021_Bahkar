package frc.robot.utils.enums;

  /**
  * Enum to control camera mode.
  */
  public enum CameraMode {
    VISION(0), CAMERA(1);

    private final int value;
    CameraMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  }