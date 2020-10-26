  package frc.robot.utils.enums;
  /**
  * Enum to control Limelight's pipeline.
  */
  public enum Pipeline {
    CLOSE(0), ZOOM(1), OFF(2);

    private final int value;

    Pipeline(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  }