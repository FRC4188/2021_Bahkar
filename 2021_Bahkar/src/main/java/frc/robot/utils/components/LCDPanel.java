// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.components;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LCDPanel extends SubsystemBase {

  private I2C LCDPanel = new I2C(I2C.Port.kOnboard, 0x27);
  /** Creates a new LCDPanel. */
  public LCDPanel() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void LCDInits() {
    writeCMD(0x03);
		writeCMD(0x03); 
		writeCMD(0x03); 
		writeCMD(0x02);
		//4 bit mode??? -- yes. Always. It's the default way of doing this for LCD displays
		writeCMD(Constants.lcdpanel.FUNCTION_SET | Constants.lcdpanel.TWO_LINE | Constants.lcdpanel.FIVExEIGHT_DOTS
            | Constants.lcdpanel.FOUR_BIT_MODE);
		writeCMD(Constants.lcdpanel.DISPLAY_CONTROL | Constants.lcdpanel.DISPLAY_ON);
		writeCMD(Constants.lcdpanel.CLEAR_DISPLAY);
		writeCMD(Constants.lcdpanel.ENTRY_MODE_SET | Constants.lcdpanel.ENTRY_LEFT);
		sleep(10);
  }

  public void sleep(int t) {
    try {
      Thread.sleep(t);
    } catch (InterruptedException e) {
    }; 
  }

  public void writeCMD(int data) {
    rawWrite(data & 0xF0);
		rawWrite((data <<4) & 0xF0);
  }

  public void rawWrite(int data) {
    LCDPanel.write(0, data | Constants.lcdpanel.BACKLIGHT_ON);
    strobe(data);
  }

  public void writeChar(int data) {
    rawWrite(Constants.lcdpanel.REGISTER_SELECT |  (data & 0xF0));
		rawWrite(Constants.lcdpanel.REGISTER_SELECT | ((data <<4 ) & 0xF0));
  }

  public void strobe(int data) {
    LCDPanel.write(0, data | Constants.lcdpanel.ENABLE);
    sleep(1);
    LCDPanel.write(0, (data & ~Constants.lcdpanel.ENABLE) | Constants.lcdpanel.BACKLIGHT_ON);
    sleep(1);
  }

  public void writeString(String string, int line) {
    switch(line) {
      case 1: 
        writeCMD(0x80);
        break;
      case 2: 
        writeCMD(0xC0);
        break;
      case 3: 
        writeCMD(0x94);
        break; 
      case 4: 
        writeCMD(0xD4);
        break;
      default: 
        return;
    }

    if (string.length() > 20) {string = string.substring(0, 20);}
  }
}
