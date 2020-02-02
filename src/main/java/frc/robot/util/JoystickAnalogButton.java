package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Button;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class JoystickAnalogButton extends Button {
  private final XboxController m_joystick;
  private final Hand m_triggerHand;
  private final double m_triggerLevel;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickAnalogButton(XboxController joystick, Hand triggerHand, double triggerLevel) {

    m_joystick = joystick;
    m_triggerHand = triggerHand;
    m_triggerLevel = triggerLevel;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return m_joystick.getTriggerAxis(m_triggerHand) > m_triggerLevel;
  }
}
