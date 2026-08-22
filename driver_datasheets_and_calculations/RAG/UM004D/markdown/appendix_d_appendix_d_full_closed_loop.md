# Appendix D - Full Closed Loop

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `21_AppD_Full_Closed_Loop.pdf` (3 pages)_

<!-- page 1 -->

Appendix D Full Closed Loop Control

> **Figure 274** — Control Structure

## Full Closed-loop Control

To execute the full closed-loop control, follow these steps.
1.
Connect auxiliary encoder (OA, OB, OZ) to the AUX port.
2. Use KNX5100C software Function List>Parameter Editor>General to
Enable full-closed loop function (x=1).
3.
Choose the source of the encoder output.
4. Configure the auxiliary encoder resolution ID171 (P1.072)
AuxFeedbackResolution.
(Gear-Ratio)
Error
Counter
Position
Loop
Gain
Speed Loop
Speed
Evaluator
Motor
Encoder
Coupling
Mechanism
Motor Feedback
Count
Low-pass filter
Auxiliary
Encoder
Encoder Output
Resolution
Encoder
Output Polarity
OA
OB
OZ
Error
Counter
OA/OB/OZ
Output Source Selection
Error protection range
between auxiliary encoder and
motor eccoder
AUX
GNUM0 , GNUM 1
Pulse
Signal
Full closed-loop
switch
P1.044
P2.060
P2.061
P2.062
P1.045
P2.000, P2.002, P2.003
P1.003
P1.046
P1.075
P1.074.Y
P1.074.X
P1.073
P1.085
Full Closed Loop Position
Error Auto Reset Threshold

<!-- page 2 -->

## Appendix D Full Closed Loop Control

5.
Set the appropriate gear ratio ID151 (P1.044), ID152 (P1.045).
6. Configure the error protection range between auxiliary encoder and
motor encoder ID172 (P1.073) MotorAndAuxFeedbackErrorLimit.
When commissioning, set (ID172 (P1.073)) with a smaller value to avoid
the motor runaway due to auxiliary encoder disconnection or polarity
configuration issue.
7.
Configure Low-pass filter time constant ID174 (P1.075)
FullHalfClosedLoopLowPassFilterTime.
8. Configure the error auto-reset condition ID677 (P1.085)
FullClosedLoopPositionErrorAutoResetThreshold.
9. Configure encoder output parameters, if necessary ID119 (P1.003.Y),
ID173 (P1.074.Y), ID153 (P1.046), ID175 (P1.076), ID179 (P1.097)).
10. Start full closed-loop control function ID173 (P1.074.X)
FullClosedLoopControlConfiguration.

<!-- page 3 -->

## Appendix D Full Closed Loop Control

Notes:
