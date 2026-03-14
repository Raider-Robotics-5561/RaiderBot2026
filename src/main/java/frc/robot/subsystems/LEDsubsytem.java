package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsubsytem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private double m_startTime;

  private final double kWhiteAuto = 21.0;
  private final double TransShift = 10.0;
  private final double Shift1 = 25.0;
  private final double Shift2 = 25.0;
  private final double Shift3 = 25.0;
  private final double Shift4 = 25.0;
  private final double Endgame = 30.0;
  // Our LED strip has a density of 120 LEDs per meter
  //private static final Distance kLedSpacing = Meter.of(120);

  public LEDsubsytem() {
    m_led = new AddressableLED(8);
    m_led.setColorOrder(ColorOrder.kBGR);
    m_ledBuffer = new AddressableLEDBuffer(58);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {

        LEDCOLORS2();
      } else if (alliance.get() == Alliance.Blue) {

        LEDCOLORS();
      }
    } else {
      // Code to handle the case where no alliance color is yet available (e.g. not
      // connected to FMS)
      System.out.println("Alliance color not available yet");
    }
  }

  public void LEDCOLORS() {
    double currentTime = Timer.getFPGATimestamp();
    // Calculate elapsed time since start
    double elapsedTime = currentTime - m_startTime;

    // Check if within the first 30 seconds
    if (elapsedTime < kWhiteAuto) {
      // Set all LEDs to Red
      setColor(Color.kWhite);
    }
    // Check if within the next 30 seconds (total 30-60 seconds)
    else if (elapsedTime < kWhiteAuto + TransShift) {
      CandyCane();
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1) {
      Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      LEDPattern base = LEDPattern.solid(Color.kBlue);
      LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(30));
      LEDPattern mode1 = base.mask(mask);
      mode1.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2) {
      LEDPattern red = LEDPattern.solid(Color.kBlue);
      LEDPattern pattern = red.blink(Seconds.of(0.2));
      pattern.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2 + Shift3) {
      LEDPattern Base = LEDPattern.solid(Color.kBlue);
      LEDPattern mask = LEDPattern.progressMaskLayer(() -> elapsedTime / 180);
      LEDPattern heightDisplay = Base.mask(mask);
      heightDisplay.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2 + Shift3 + Shift4) {
      Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      LEDPattern base = LEDPattern.solid(Color.kBlue);
      LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(30));
      LEDPattern mode1 = base.mask(mask);
      mode1.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2 + Shift3 + Shift4 + Endgame) {
      LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
      LEDPattern Endgame1010 = steps.blink(Seconds.of(0.5));
      Endgame1010.applyTo(m_ledBuffer);
    } else {
      LEDPattern black = LEDPattern.solid(Color.kBlack);
      black.applyTo(m_ledBuffer);
    }

    // Update the LEDs with the buffer data
    m_led.setData(m_ledBuffer);
  }

  public void LEDCOLORS2() {
    double currentTime = Timer.getFPGATimestamp();
    // Calculate elapsed time since start
    double elapsedTime = currentTime - m_startTime;

    // Check if within the first 30 seconds
    if (elapsedTime < kWhiteAuto) {
      // Set all LEDs to Red
      setColor(Color.kWhite);
    }
    // Check if within the next 30 seconds (total 30-60 seconds)
    else if (elapsedTime < kWhiteAuto + TransShift) {
      CandyCane2();
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1) {
      Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      LEDPattern base = LEDPattern.solid(Color.kRed);
      LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(30));
      LEDPattern mode1 = base.mask(mask);
      mode1.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2) {
      LEDPattern bed = LEDPattern.solid(Color.kRed);
      LEDPattern pattern = bed.blink(Seconds.of(0.2));
      pattern.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2 + Shift3) {
      LEDPattern Base = LEDPattern.solid(Color.kRed);
      LEDPattern mask = LEDPattern.progressMaskLayer(() -> elapsedTime / 180);
      LEDPattern heightDisplay = Base.mask(mask);
      heightDisplay.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2 + Shift3 + Shift4) {
      Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      LEDPattern base = LEDPattern.solid(Color.kRed);
      LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(30));
      LEDPattern mode1 = base.mask(mask);
      mode1.applyTo(m_ledBuffer);
    } else if (elapsedTime < kWhiteAuto + TransShift + Shift1 + Shift2 + Shift3 + Shift4 + Endgame) {
      LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));
      LEDPattern Endgame1010 = steps.blink(Seconds.of(0.5));
      Endgame1010.applyTo(m_ledBuffer);
    } else {
      LEDPattern black = LEDPattern.solid(Color.kBlack);
      black.applyTo(m_ledBuffer);
    }
    // Update the LEDs with the buffer data
    m_led.setData(m_ledBuffer);

  }

  public void CandyCane() {
    boolean isWhite = true;
    long lastToggleTime = 0;
    long interval = 500;
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastToggleTime > interval) {
      lastToggleTime = currentTime;
      isWhite = !isWhite;
    }
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (isWhite) {
        m_ledBuffer.setRGB(i, 255, 255, 255);
      } else {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }
    }
  }

  public void CandyCane2() {
    boolean isWhite = true;
    long lastToggleTime = 0;
    long interval = 500;
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastToggleTime > interval) {
      lastToggleTime = currentTime;
      isWhite = !isWhite;
    }
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (isWhite) {
        m_ledBuffer.setRGB(i, 255, 255, 255);
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
    }
  }

  private void setColor(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }
}