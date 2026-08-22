# Appendix E - Scope Function

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `22_AppE_Scope_Function.pdf` (10 pages)_

<!-- page 1 -->

## Appendix E

Use the Scope Function in KNX5100C Software
The Scope option provides you with a personal computer-based digital
oscilloscope function for real-time drive data monitoring. You can use two
scope modes (8K and 16K) to measure and monitor many signals quickly
without oscilloscope equipment.
Get Started
To open the Scope dialog box, select Scope from the Function list.
Topic
Page
Get Started

Scope Functions

Quick Setup of Communication Channels

Select Communication Channels

Enable Stop Condition

FFT Display and Show RMS Value

Fine-Tune the Scope

Set Preferences

Use Popup Menu for Save Options

<!-- page 2 -->

Appendix E Use the Scope Function in KNX5100C Software
Scope Functions
To execute Scope functions, click icons on the toolbar.
•
Save File (*.scp): Save the waveform displays on the screen as a scope file
(*.scp) or other files in several kinds of format.
•
Open File (*.scp): Open and read the scope file (*.scp) and display the
waveform displays on the screen.
•
Show all channels with same scaling: Display the channel data with the
same scaling/axis on the screen.
•
Show all channels with different scaling: Display the channel data with
different scaling/axis on the screen.
•
Zoom in (F5). Zoom in to the waveform displays on the current screen.
•
Zoom out (F6). Zoom out of the waveform displays on the current screen.
•
Previous screen (F7). Display the previous screen.
•
Optimize the current view: Optimize the display in the current screen.
•
Clear Screen. Clear the waveform displays on the current screen.
•
Screen second switch (20 <-> 120). It is used to select the data buffer (Xaxis) that the screen can display one time. When selecting 20, it indicates
that the screen can display 20 seconds of data buffer for one time.
•
Run: start to execute the data logging against time.
•
Quick tool: select the Quick Setup from the Quick tool template.
•
Add: add the current channel setting to Quick tool template.
•
Update: update the current channel setting to Quick tool template.
•
Delete: delete the selected Quick Setup from Quick tool template.
Quick Setup of
Communication Channels
Use the pull-down menu to select monitor option from Quick tool template
and do the quick setup of Communication Channels.
•
E005 Analysis - Quick setup for E005 (Regeneration Error) Analysis.
•
E009 Analysis - Quick setup for E009 (Excessive deviation of Position
Command) Analysis.
•
E02C Analysis - Quick setup for E02C (Drive Overload) Analysis.
•
Tuning Gain Analysis - Quick setup for Gain Analysis.
•
Tuning Vibration Analysis - Quick setup for Vibration Analysis.
•
Pulse command analysis - Quick setup for Pulse command analysis.
•
Speed-torque Analysis - Quick setup for Speed-torque Analysis.

<!-- page 3 -->

Appendix E Use the Scope Function in KNX5100C Software
Select Communication
Channels
The scope speed determines the available channels. After you choose the
communication speed, the system disables the unavailable channels
automatically.
•
CH (Channel) (1) - To display data on the screen, check the channel that
you want to view.
•
32 bit (2) - This option is used to select the data length of the channel.
- When the checkbox is checked, it indicates that it is a 32-bit data. At
this time, two 16-bit data (the data of CH1 and CH3) is combined into a
32-bit data and CH3 is disabled. By the same logic, the data of CH2 and
CH4 is combined into a 32-bit data channel and CH4 is disabled.
- When the checkbox is not checked, it indicates it is 16-bit data. You can
enable each channel.
•
Color Selection (3) - You can choose the channel display color by
preference. When you click, a color selection dialog box appears and you
can choose your favorite color.
•
Data (4) - If there is data on the screen, when you move your mouse over
the data, the current data value of the point where the mouse is located is
shown.
•
Relative Value (Rel. val.) (5) - If there is data on the screen, when you
move your mouse over the data, the data value relative to the value of the
starting point is shown.
Double click the Data field to display the data with BIT or HEX format.

<!-- page 4 -->

Appendix E Use the Scope Function in KNX5100C Software
•
General - From the pull-down menu, choose General. The General setting
lets you choose what to monitor.
•
Parameters - From the pull-down menu, choose Parameters.
•
User Array - From the pull-down menu, choose User Array. Set the array
between 0…63 for the array you want to monitor.

<!-- page 5 -->

Appendix E Use the Scope Function in KNX5100C Software
•
CH 5…8: This function is only available for 8K baud-rate enabled. You can
observe all eight channels of data. Channels 5…8 function the same as
Channels 1…4.

<!-- page 6 -->

Appendix E Use the Scope Function in KNX5100C Software
Enable Stop Condition
When Enable Stop Condition option is selected, after Run is pressed, you can
select one channel and stop its monitoring operation after a period.
1.
Check Enable Stop Condition.
2. Select the channel.
3.
Select the logical condition (>=, =, and <=), add a value that determines
the threshold for when the monitoring operation is to stop, and click Set.
4. Add a value for ‘Continue after condition satisfied’.
This option is used to set the time (lasting time) for which the digital
scope collects data after the stop conditions are met.
For example, select CH2, select ‘=’ and set the value to 1000, then set the
continue time to 2000 ms. Once the value reaches 1000, data is collected
for another 2000 ms.

<!-- page 7 -->

Appendix E Use the Scope Function in KNX5100C Software
FFT Display and Show RMS
Value
This option is used to display the frequency spectrum of the waveforms. From
the Condition tab, you can select FFT Display and Show RMS Value.
When you check FFT Display, use the zoom in/zoom out tools or use the mouse
to drag a rectangle area and drop it on the screen to display the frequency
spectrum of the waveforms.
When you check Show RMS Value, the RMS value of each channel is also
displayed.

<!-- page 8 -->

Appendix E Use the Scope Function in KNX5100C Software
•
The X-axis represents the frequency and the unit is Hz.
•
The Y-axis represents the strength of the signal, which has no absolute
unit but is a relative value.
•
The icons on the FFT Display toolbar function the same as the icons on
the Scope toolbar.
•
Double-click any point of the FFT Display screen and all points display.
•
You can also use the mouse to drag a rectangle and drop it on the screen
to display the frequency spectrum of the waveforms.
•
The title bar of FFT Display window, such as FFT: 32768 Pts, indicates the
data number of the selected area. The larger the number, the better the
resolution (DPI™). We recommended a value of at least 512 or higher.
•
The image of the FFT Display screen can also be saved as a picture (*.bmp
file). Place the cursor on the FFT Display screen, right-click the mouse,
and choose ‘Save as picture (*.bmp)’.
Fine-Tune the Scope
When the other functions cannot meet your requirements, you can enable the
Fine-Tuning function.

<!-- page 9 -->

Appendix E Use the Scope Function in KNX5100C Software
•
When the Enable Single Channel checkbox is checked, it indicates that
Fine-Tuning function is enabled.
•
When the Channel checkboxes are checked, only the waveform displays
of the selected channels can be zoomed in/out and moved.
•
When all Channel checkboxes are checked, the waveform displays of all
channels can be zoomed in/out and moved.
•
Zoom Time-axis only - If this option is not selected, the values of Time
axis do not change when you zoom in/out and move the waveform
displays.
•
Zoom Value-axis only - If this option is not selected, the values of Value
axis do not change when you zoom in/out and move the waveform
displays.
•
Align Time axis based on this CH - When this button is clicked, the
system changes the time of other channels and make it the same as the
time of the selected channel.
•
Specify Display Range - The start and end values are the start and end
points of the data. To convert these values into time, multiply by the time
unit. For example:
- Time unit is 0.125 ms
- Start point is 100, therefore, 100 * 0.125 = 12.5 ms
- Endpoint is 2000, therefore 2000 * 0.125 = 250 ms
Set Preferences
•
Keep current Max/Min while running - The scope remains the current
Max/Min value after the user pressing RUN if this setting is checked. The
system uses the default Max/Min value if this option is not checked.
•
Show Grid Line - The scope screen shows the grid line when selecting this
option. The square button on left is to select the color of the grid line.
•
Auto adjust the scope display - The system adjusts the size of scope
automatically according to screen resolution and selected condition.
•
Show saving preference options - The options that appear when you
right-click on the screen.

<!-- page 10 -->

Appendix E Use the Scope Function in KNX5100C Software
Use Popup Menu for Save
Options
When you right-click the mouse on the screen, the following popup menu
appears.
•
Save as a picture(*.bmp) Save the signal data as a picture (*.bmp file).
•
Save Scope Data as Text File (*.txt) Save all signal data as a text file (*.txt
file).
•
Save Data of Current Display as Text (*.txt) Save the signal data on the
current display as a text file (*.txt file).
