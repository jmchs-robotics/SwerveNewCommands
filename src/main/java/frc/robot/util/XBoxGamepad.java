package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Wrapper to create the XBoxController and all of its buttons at once. Buttons are only created (new'd) when they are first accessed.
 */
public class XBoxGamepad {
  private XboxController m_joystick;

  private JoystickButton m_A, m_B, m_X, m_Y, m_bumperLeft, m_bumperRight, m_stickLeft, m_stickRight, m_start, m_back;
  private JoystickAnalogButton m_leftTrigger, m_rightTrigger;
  private JoystickAnalogButton m_leftJoystickX, m_leftJoystickY;
  private JoystickAnalogButton m_rightJoystickX, m_rightJoystickY;
  private POVButton m_N, m_NE, m_E, m_SE, m_S, m_SW, m_W, m_NW, m_POVunpressed;

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

  /**
   * Get an {@link Axis} as a button with the default trigger level.
   * @param axis The {@link Axis} to generate a button from.
   * @return The first {@link JoystickAnalogButton} created for the axis on this joystick.
   */
  public JoystickAnalogButton getTriggerAsButton(Axis axis){
    switch(axis){
      case kLeftTrigger:
        if(m_leftTrigger == null)
          m_leftTrigger = new JoystickAnalogButton(m_joystick, axis, m_defaultTriggerLevel);
        return m_leftTrigger;
      case kRightTrigger:
      if(m_rightTrigger == null)
          m_rightTrigger = new JoystickAnalogButton(m_joystick, axis, m_defaultTriggerLevel);
        return m_rightTrigger;
      case kLeftX:
        if(m_leftJoystickX == null)
          m_leftJoystickX = new JoystickAnalogButton(m_joystick, axis, m_defaultTriggerLevel);
        return m_leftJoystickX;
      case kRightX:
        if(m_rightJoystickX == null)
          m_rightJoystickX = new JoystickAnalogButton(m_joystick, axis, m_defaultTriggerLevel);
        return m_rightJoystickX;
      case kLeftY:
        if(m_leftJoystickY == null)
          m_leftJoystickY = new JoystickAnalogButton(m_joystick, axis, m_defaultTriggerLevel);
        return m_leftJoystickY;
      case kRightY:
        if(m_rightJoystickY == null)
          m_rightJoystickY = new JoystickAnalogButton(m_joystick, axis, m_defaultTriggerLevel);
      default:
        return null;
    }
  }

  /**
   * Get a POV as a button.
   * @param angle The angle [0, 360) of the POV to tie a button to.
   * @return The first {@link POVButton} created for that POV angle.
   */
  public POVButton getPOVAsButton(int angle){
    switch(angle){
      case 0:
        if(m_N == null) m_N = new POVButton(m_joystick, angle);
        return m_N;
      case 45:
        if(m_NE == null) m_NE = new POVButton(m_joystick, angle);
        return m_NE;
      case 90:
        if(m_E == null) m_E = new POVButton(m_joystick, angle);
        return m_E;
      case 135:
        if(m_SE == null) m_SE = new POVButton(m_joystick, angle);
        return m_SE;
      case 180:
        if(m_S == null) m_S = new POVButton(m_joystick, angle);
        return m_S;
      case 225:
        if(m_SW == null) m_SW = new POVButton(m_joystick, angle);
        return m_SW;
      case 270:
        if(m_W == null) m_W = new POVButton(m_joystick, angle);
        return m_W;
      case 315:
        if(m_NW == null) m_NW = new POVButton(m_joystick, angle);
        return m_NW;
      case -1:
        if(m_POVunpressed == null) m_POVunpressed = new POVButton(m_joystick, angle);
        return m_POVunpressed;
      default:
        return null;
    }
  }

  /**
   * Get an {@link Axis} as a button with a custom trigger level.
   * @param axis The {@link Axis} to generate a button from.
   * @param triggerLevel A number between -1 and 1. If the axis moves beyond that level, the button will trigger.
   * @return The first {@link JoystickAnalogButton} created for the axis on this joystick.
   */
  public JoystickAnalogButton getTriggerAsButton(Axis axis, double triggerLevel){
    switch(axis){
      case kLeftTrigger:
        if(m_leftTrigger == null)
          m_leftTrigger = new JoystickAnalogButton(m_joystick, axis, triggerLevel);
        return m_leftTrigger;
      case kRightTrigger:
      if(m_rightTrigger == null)
          m_rightTrigger = new JoystickAnalogButton(m_joystick, axis, triggerLevel);
        return m_rightTrigger;
      case kLeftX:
        if(m_leftJoystickX == null)
          m_leftJoystickX = new JoystickAnalogButton(m_joystick, axis, triggerLevel);
        return m_leftJoystickX;
      case kRightX:
        if(m_rightJoystickX == null)
          m_rightJoystickX = new JoystickAnalogButton(m_joystick, axis, triggerLevel);
        return m_rightJoystickX;
      case kLeftY:
        if(m_leftJoystickY == null)
          m_leftJoystickY = new JoystickAnalogButton(m_joystick, axis, triggerLevel);
        return m_leftJoystickY;
      case kRightY:
        if(m_rightJoystickY == null)
          m_rightJoystickY = new JoystickAnalogButton(m_joystick, axis, triggerLevel);
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
