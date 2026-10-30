=======================================
VW e-Golf: Climate & Heating web plugin
=======================================

**Remote climate, window and seat heating for the VW e-Golf (VWEG), plus a read-only status overview**

- :download:`vwegolf-climate.htm`

Requires the VWEG vehicle module with BatteryControl climate support (the ``xvg windowheat``,
``xvg seatheat`` and ``xvg charge profile`` commands).

--------
Features
--------

- **Readiness strip:** SOC, 12V voltage, 12V charging, plug state, parked time, awake/asleep.
- **Climate:** start/stop remote climate (``climatecontrol on|off``) with cabin, target and outside
  temperatures, BCU climate mode and state, and battery draw.
- **Target temperature:** 15.5–30.0 °C slider; "Save to car" writes charge profile 0
  (``xvg charge profile set 0 temp <°C>``), the same value the MIB shows.
- **Window heating:** front, rear or both (``xvg windowheat on|off front|rear|both``).
- **Seat heating:** driver, passenger or both (``xvg seatheat on|off fl|fr|front``).
- **Vehicle status:** read-only cards for vehicle, HV battery, charging and module metrics.
- **Last command:** the command sent and the module's reply; refusals ("another command is in
  progress") are highlighted.
- **BCU details (debug):** raw BCU climate, charge and plug metrics, and a switch that shows the
  metric name under each value.

Opening the page never wakes the car: all status comes from metrics the module already has. When the
car is asleep (``v.e.awake`` = no), every button asks for confirmation before sending a command that
wakes it.

------------
Installation
------------

1. Save :download:`vwegolf-climate.htm` to your computer.
2. In the OVMS web UI open Config → Web Plugins, add an entry named ``vwegolf-climate`` of type
   Page, save the list, then click its Edit button. Recommended setup:

  - Page:          ``/usr/vwegolf-climate``
  - Label:         Climate & Heating
  - Menu:          Vehicle
  - Authorization: Cookie

3. Paste the file content into the editor and save. The page appears in the Vehicle menu.

To update, open the same entry's editor and paste the new version.

-----
Notes
-----

- Climate, window and seat heating commands are fire-and-forget: the reply only says the request was
  accepted. The status pills and badges update as the BCU reports, roughly every 13 s while awake.
- Seat heating bits stay stored in profile 0 after a seat command (the module says so in its reply);
  ``xvg seatheat clear`` in the shell removes them.
- The target temperature write blocks while the car wakes and answers (up to about 22 s).
- Scheduled climate preconditioning needs a firmware change (registering the framework's
  precondition schedule page) and is not part of this plugin.
