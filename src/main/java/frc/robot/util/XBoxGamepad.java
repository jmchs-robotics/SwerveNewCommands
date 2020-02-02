package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Wrapper to create the XBoxController and all of its buttons at once. Buttons are only created (new'd) when they are first accessed.
 */
public class XBoxGamepad {
  private XboxController m_joystick;

  private JoystickButton m_A, m_B, m_X, m_Y, m_bumperLeft, m_bumperRight, m_stickLeft, m_stickRight, m_start, m_back;
  private JoystickAnalogButton m_left, m_right;

  private final double m_defaultTriggerLevel;

  public XBoxGamepad(int id) {
    m_joystick = new XboxController(id);
    m_defaultTriggerLevel = 0.5;
  }

  public XBoxGamepad(int id, double triggerLevel){
    m_joystick = new XboxController(id);
    m_defaultTriggerLevel = triggerLevel;
  }

  /**
   * Get the JoystickButton. Buttons are created the first time they are
   * referenced.
   * 
   * @param button The enum representing the button to get.
   * @return A JoystickButton object representing the button requested.
   */
  public JoystickButton getButton(Button button) {
    switch (button) {
    case kA:
      if (m_A == null)
        m_A = new JoystickButton(m_joystick, Button.kA.value);
      return m_A;
    case kB:
      if (m_B == null)
        m_B = new JoystickButton(m_joystick, Button.kB.value);
      return m_B;
    case kX:
      if (m_X == null)
        m_X = new JoystickButton(m_joystick, Button.kX.value);
      return m_X;
    case kY:
      if (m_Y == null)
        m_Y = new JoystickButton(m_joystick, Button.kY.value);
      return m_Y;
    case kBack:
      if (m_back == null)
        m_back = new JoystickButton(m_joystick, Button.kBack.value);
      return m_back;
    case kBumperLeft:
      if (m_bumperLeft == null)
        m_bumperLeft = new JoystickButton(m_joystick, Button.kBumperLeft.value);
      return m_bumperLeft;
    case kBumperRight:
      if (m_bumperRight == null)
        m_bumperRight = new JoystickButton(m_joystick, Button.kBumperRight.value);
      return m_bumperRight;
    case kStart:
      if (m_start == null)
        m_start = new JoystickButton(m_joystick, Button.kStart.value);
      return m_start;
    case kStickLeft:
      if (m_stickLeft == null)
        m_stickLeft = new JoystickButton(m_joystick, Button.kStickLeft.value);
      return m_stickLeft;
    case kStickRight:
      if (m_stickRight == null)
        m_stickRight = new JoystickButton(m_joystick, Button.kStickRight.value);
      return m_stickRight;
    default:
      return null;
    }
  }

  public JoystickAnalogButton getTriggerAsButton(Hand hand){
    switch(hand){
      case kLeft:
        if(m_left == null)
          m_left = new JoystickAnalogButton(m_joystick, hand, m_defaultTriggerLevel);
        return m_left;
      case kRight:
      if(m_right == null)
          m_right = new JoystickAnalogButton(m_joystick, hand, m_defaultTriggerLevel);
        return m_right;
      default:
        return null;
    }
  }

  public double getLeftX() {
    return m_joystick.getX(Hand.kLeft);
  }

  public double getLeftY() {
    return m_joystick.getY(Hand.kLeft);
  }

  public double getLeftTrigger() {
    return m_joystick.getTriggerAxis(Hand.kLeft);
  }

  public double getRightX() {
    return m_joystick.getX(Hand.kRight);
  }

  public double getRightY() {
    return m_joystick.getY(Hand.kRight);
  }

  public double getRightTrigger() {
    return m_joystick.getTriggerAxis(Hand.kRight);
  }

  public int getPOV() {
    return m_joystick.getPOV();
  }

  public void setRumble(RumbleType type, double value) {
    m_joystick.setRumble(type, value);
  }

  public XboxController getController(){
    return m_joystick;
  }
}
