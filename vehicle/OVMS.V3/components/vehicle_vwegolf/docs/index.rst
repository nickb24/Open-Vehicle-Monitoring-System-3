
=========
VW e-Golf
=========

Vehicle Type: **VWEG**

This vehicle type supports the VW e-Golf.
Connection is made via the J533 gateway module, preferably using a harness
adapter that plugs into the existing connector.

-------
Adapter
-------

The adapter connects to the J533 gateway harness connector, giving access to
the Comfort CAN (KCAN) and Powertrain CAN (FCAN) buses.  Everything the module
reads arrives on KCAN: the J533 gateway rebroadcasts the powertrain/HV frames
(VIN, gear selector, SoC, pack current/voltage, speed) onto KCAN, so a single
KCAN tap sees them all.  The module therefore registers KCAN only; FCAN is
present on the adapter but not used.

The adapter extends the car harness — pins are connected straight through,
with the CAN pairs run as twisted pair (e.g. from CAT6 cable).  The
connector part number is VW 8E0972420 (buy both male and female).

.. image:: j533_to_ovms.svg

.. image:: adapter.jpg
    :width: 480px

------------
Installation
------------

Installation does not require removal of any trim pieces.
The J533 gateway module is located under the steering wheel, above the pedals.
Position yourself face-up under the dashboard — you should see the gateway
module with its red connector from there.

.. image:: installing_position.jpg
    :width: 480px

.. image:: gateway.jpg
    :width: 480px

Unplug the red connector, plug in the adapter, connect the OVMS module to the
adapter, and tuck it somewhere in the dashboard.

-------------------
Basic Configuration
-------------------

After selecting the VW e-Golf vehicle module, data should start flowing in
once the car is awake.

----------------
Support Overview
----------------

=========================== ==============
Function                    Support Status
=========================== ==============
Hardware                    Any OVMS v3 (or later) module. Vehicle support: 2014–2020
Vehicle Cable               Harness adapter for J533 gateway module (see above)
GSM Antenna                 1000500 Open Vehicles OVMS GSM Antenna (or compatible)
GPS Antenna                 1020200 Universal GPS Antenna (or compatible)
SOC Display                 Yes
Range Display               Yes
Cabin Pre-heat/cool Control Yes
GPS Location                Yes (decoded from KCAN frame 0x486)
Speed Display               Yes
Temperature Display         Yes (see list of metrics below)
BMS v+t Display             No
TPMS Display                No
Charge Status Display       Yes
Charge Interruption Alerts  No
Charge Control              Charge-current limit, start/stop (see below)
Lock/Unlock Vehicle         No
Valet Mode Control          No
12V Charging Status         Yes (``v.e.charging12v``, see below)
Doors, Windows, Lights      Doors, locks, window positions, bonnet, side lights (see below)
Others
=========================== ==============

-------------------
12V Charging Status
-------------------

* ``v.e.charging12v`` is set while the DC-DC converter charges the 12V battery. The e-Golf has no alternator, so
  this is whenever the HV system is up: the HV-system-active bit of KCAN frame 0x187 (bit 15, ``HV_Bordnetz_aktiv``
  of ``Motor_EV_01`` in opendbc), HV charging in progress, or remote climate running. It clears when KCAN goes silent
* OVMS Power Management (``power`` config) relies on this metric to tell whether the 12V battery is being charged

-------------------------
Doors, Windows and Lights
-------------------------

All read from the comfort bus while the car is awake. The values are kept while the car sleeps (windows and locks
cannot change then).

================================ =========================================================================
Metric                           Meaning
================================ =========================================================================
``v.d.fl`` … ``v.d.rr``          Door open (standard metrics), ``v.d.trunk`` tailgate, ``v.d.hood`` bonnet
``v.e.locked``                   All doors locked (key fob, keyless entry or central-lock button)
``xvg.v.d.lock.fl`` … ``rr``     Door locked, per door
``xvg.v.d.window.fl`` … ``rr``   Window position in % open (0 = closed, 100 = fully open, -1 = not seen yet)
``xvg.v.e.sidelights``           Light switch in the side-light position
``xvg.v.rhd``                    Steering side reported by the car: ``yes`` = right-hand drive
================================ =========================================================================

* The car's unlock setting (infotainment menu: all doors, single door or vehicle side) decides which doors a key
  fob press or a door handle unlocks, and on a car locked from inside a door's inside handle unlocks only that
  door. Each door reports its own lock state: the per-door lock metrics show which doors are unlocked, and
  ``v.e.locked`` reads ``no`` as soon as one door is unlocked
* Left / right: the car reports doors, windows and locks per driver / passenger side. OVMS maps them to left / right
  using ``xvg.v.rhd``. Verified on a left-hand-drive car and on a right-hand-drive car (UK); please report
  anything shown on the wrong side in a
  `GitHub issue <https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/issues>`_

---------------------
Cabin Climate Control
---------------------

* Temperature setpoint is set in the car's infotainment system (e-Manager)
* Climatise on battery power is force-enabled
* OVMS may show the car as charging/plugged-in even if it is not charging/plugged-in
* Climate control heats or cools the cabin only. Window heating is a separate remote function, as in the car's own
  remote control: ``xvg windowheat on`` / ``off`` heats the windscreen and rear window, on its own or while
  climate is running. ``xvg windowheat on|off front`` or ``rear`` switches one window only (experimental; the
  car's own remote control always switches both)
* Window and seat heating commands count as done only once the car reports the heater running (for ``off``: no
  longer running); otherwise they fail with "not started by the car". If the car starts only some of the
  requested heaters, an info notification names the ones it did not start
* Every climate start clears any window heating option stored in the car. The former
  ``xvg charge profile set 0 frontwindow`` setting has been removed
* With window heating included in a climate start, the windscreen only heated below 4 °C outside, as set in the
  HVAC module (08) adaptation "IDE07303-Rem.start funct.:activ.temper. of windshield defrost.heat." (default 4 °C).
  Whether this also applies to ``xvg windowheat`` is not yet known
* The outside temperature ``v.e.temp`` comes from the outside sensor's unfiltered value while the car is switched off,
  and from the instrument cluster's filtered value (the one on the dashboard) while it is on, as in VW's own
  telematics unit
* ``xvg seatheat on`` / ``off`` ``[front|fl|fr]`` is an experimental remote seat heating command. The seat setting
  stays stored in the car until ``xvg seatheat clear``. On right-hand-drive cars ``fl`` is the passenger seat
* The metrics ``xvg.v.env.heated_front_window`` and ``xvg.v.env.heated_rear_window`` show whether window heating
  is active as part of remote heating. They do not reflect the heater buttons on the dashboard
* The window heaters use additional energy, which reduces range when running on battery power

--------------
Charge Control
--------------

Charge control only applies to the default settings in the car, not the user defined charge locations/departure timers.

This means:

* Current limit does not apply by default if you have departure timers enabled because the car will not use the default settings.
* The moment you use the charge control feature the car will switch to the default charge profile (profile 0,
  "Optionen") and charge immediately with its settings, including its charge limit. Where profile 0 sets no limit
  (``xvg charge profile list`` shows ``-`` under TgtSoC), the car charges to 100% SoC
* You cannot switch back to your departure timer via OVMS, press the delayed charging button on the charge port

Furthermore to make it compatible with climate control:

* When climate control is started while charging you have to turn off climate control first before you can stop charging
* Enabling climate control while plugged in will cause the car to start charging again
* To avoid climate preconditioning from starting when using the charge start command while unplugged the flag "climatise on battery" is cleared

Charge current limit only supports a few set values: 5A, 10A, 13A or 32A (= Max)
