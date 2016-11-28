# TS_EnergyMonitor
Upload Energy Monitor data to Thingspeak

I have dialled in some calibration parameters that work for me in RealPower mode in the attached file please use them as a starting point. The procedure for calibration is summarised below:

1. Calibrate voltage to your line voltage first. With default parameters there is still a ramp off near the beginning, wait for voltage to settle before calibrating. You can use Arduino IDE's built in Serial plotter to see the ramp.
**New_VCAL = Old_VCAL x Target voltage/ Observed voltage** . i.e. is target voltage is smaller than observed voltage vcal will become smaller and vice-versa

2. Calibrate power offset . The ADS1115 Arduino library from Adafruit has a 8millisecond delay between one shot readings. ADS1115 can read in continuous mode at 860sps which would be nice but we don't have ready made implementations of that. So we will calibrate. There will be a non-zero power even with zero load, we are going to add a power offset to subtract this and bring it to zero.

3. Calibrate current multiplier ICAL. I do this using a 116W halogen lamp. This is a purely resistive load and should have power factor of one. However due to lag issues mentioned above power factor measurement for the ADS1115 meter is not great. **New_ICAL = Old_ICAL X 116W (Target power) / Observed power**. So if observed power is less than target ICAL becomes smaller and vice versa. If the observed power is negative flip the CT around and restart from step 2.

4. Recalibrate power offset. This as an iterative refinement of the offset due to change of ICAL. Switch bulb off and change offset to obtain zero reading at no-load.

5. Iterate steps 2-4 until calibrated to reasonable accuracy.

6. With the lamp on have a look at power factor . It possibly exceeds one, set reading as the **PCAL** value to bring the maximum power factor down to one. PCAL may also need the offset / scale treatment given to the power, not tested yet.

The meter should now be resonably accurate. I would appreciate if you gave me any feedbacks and refinements of this process from your personal experience. I will also record a corresponding video.

