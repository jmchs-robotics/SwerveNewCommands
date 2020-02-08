package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class JoystickAnalogButton extends Button {
  private final XboxController m_joystick;
  private final Axis m_triggerAxis;
  private double m_triggerLevel;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param triggerAxis The input axis.
   * @param triggerLevel The signed level the trigger has to pass for the button to evaluate to true. If triggerLevel is less than 0,
   *                      get() returns true if the input is less than the trigger level. Else, the input returns true if the input is
   *                      greater than or equal to the trigger level.
   */
  public JoystickAnalogButton(XboxController joystick, Axis triggerAxis, double triggerLevel) {
    m_joystick = joystick;
    m_triggerAxis = triggerAxis;
    m_triggerLevel = triggerLevel;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button.
   */
  @Override
  public boolean get() {
    if(m_triggerLevel >= 0)
      return m_joystick.getRawAxis(m_triggerAxis.value) >= m_triggerLevel;
    return m_joystick.getRawAxis(m_triggerAxis.value) < m_triggerLevel;
  }

  /**
   * Get the trigger level for this button.
   */
  public double getTriggerLevel(){
    return m_triggerLevel;
  }
  /**
   * Change the trigger level for this button.
   */
  public void setTriggerLevel(double newLevel){
    m_triggerLevel = newLevel;
  }
}
