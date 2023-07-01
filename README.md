# DS4-cirocco-retrofit

New updated sketch see Cirocco_retrofit\Cirocco_retrofit_completev1.ino:



Sketch by Nico1080
This sketch was tested on a DS4, and should also work on any C4 gen 2 (hatchback, sedan, etc)

Look at my profile to see all modification I brought to the car:
https://www.drive2.ru/r/citroen/ds4/609447751477897832/

Many thanks to:
- Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration
- Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)
- Infizer for many items  (https://github.com/infizer91/can_extender  , https://www.drive2.ru/users/infizer/ , https://telecoder.ru/index.php )
And all I have forgotten

Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it)

The function of this sketch:
- Make SAM (Surveillance Angles Mort, blind spots monitor) NAC toogle switch work
- AAS (Aide Au Stationement, parking sensor) need BSI to be coded to listen to NAC and not cluster anymore (some code could be wrote to have both working)
- Speed limit display from CVM and NAV data (see comment)
- Fix SAM light on cirocco (frame 0x321)  (Thanks to Infizer)
- Fix animation on cirocco (frame 0x236)  (Thanks to Infizer)
- Display green/orange line on cirrocco (AFIL and LKA need to be activated in cirocco) (Thanks to Infizer)
- Make AFIL indicator flash when lines are being crossed without turn indicator (see comment)(Thanks to Infizer)
- Deactivate Stop & Start 5s after engine start
- Fix MEM returning to default value for all setting (40km/h) (Thanks to Infizer)
- Double pressing MEM will set and activate cruise control/ speed limiter to a corrected speed value (offset+ rain, see comment)
- Physical connection with AAS/ECO/SAM button/led to make them work again and add other function (see comment)
- Display front/rear camera (VisioPark2) by pressing AAS/ECO button (see comment)
- Display front camera(VisioPark2) when front obstacle is detected by AAS (see comment)
- Change Cirocco ambiance/theme by pressing the ESC button on the wheel (see comment)

For speed limit display it follow this logic:
- show end of speed-limit sign for 3s when it is read by CVM
- show CVM speed when it is reliable (red sign)
- keep showing CVM speed in red sign after CVM loose reliability. Only if NAV speed have the same value and has not changed since it lost CVM reliability (CVM reliability is easily lost so I "extend" it)
- show NAV speed (grey sign)
- if no NAV speed is available it display last read sign (grey sign and remove 1km/h. Example last sign=70km/h display 69 in grey sign)
- If no previous sign (and no nav speed) it display nothing

For AFIL  (Alerte Franchissement Involontaire de Ligne, line detection):
- Display green orange line as soon as they are detected at any time (require LKA activated in cirocco)
- Flash AFIL light when one line become orange and if one(and not 2) turning indicator is ON  and when speed is over 50km/h (require AFIL activated in Cirocco)
- When left/right lines are both detected: margin calculation for AFIL threshold (large road= big margin small road= small margin) default value is 1300.

For Stop and start deactivation:
- It require NAC toggle key working for stop and start.  BSI need to be coded to telematic in engine menu (Type d'acquisition du contacteur stop and start) , other choices are BSI (original value on my DS4) and cluster.
- If stop & start is not already deactivated, it will send request (Id 1A9) to do it 5s after engine start (rpm>500) and check if it worked (if it didn't, it will try again)

For MEM FIX:
- sketch  will send 19b(limit) & 1DB (cruise) on CAN-DIV after ignition with speed value (BSI will send the same frame several times (6 for limit, 3 for cruise) sorted in ascending order
- Some code could be written to check if driver changed setting and store new values inside EEPROM

For setting cruise control/ speed limiter:
- the sketch will program the speed value into the MEM setting and emulate the needed button press (pause, MEM etc) to activate it.
- The wheels button need only to be set the cruise or limiter mode.
- The set speed is offseted by 2 or 3km/h (between 70/79 and above 80)
- If wiper are active (auto mode) the speed will also be decrease by 20 for 130 and 10 for 90/110 (French speed limit are lowered when raining)

For AAS/ECO/SAM button connection: 
- Button pin need to connected to arduino input pin (no need for resistor as INPUT_PULLUP is activated
- When button are pushed sketch will send request on ID 1A9 (ECO and AAS, NAC emulation, BSI need to be coded to listen to NAC) or 217 (SAM, Cluster emulation as BSI can not be coded for other source)
- For AAS/ECO/SAM led connection: a level shifter is required (led use 12v logic and arduino is only 5v) I used a UDN2981.
- Sketch listen to ID 227 (ECO and AAS) and 2D1 (SAM). It will turn on the led only if ignition is on.
- For button backlight I kept the original wire from car(265 generated by BSI, connected on button pin 4).  Some code could be written to integrate it on arduino and avoid extra wiring
- See https://www.drive2.ru/l/633915458608714452/

For VisioPark2 the sketch will:
- Show front video when front obstacle is detected by AAS
- Show/hide front video when AAS button is short pressed (<800ms)
- Show/hide rear video when ECO button is short pressed (<800ms)
- Long press (>800ms) on AAS/ECO button will make normal operation (ID 1A9 ECO and AAS NAC emulation)
- In any case video will automatically:
   - 	Disappear when going over 25km/h (setting built inside NAC, no way around it)
   - 	Rear video will show when rear gear is engaged 	


For changing Cirocco ambiance/theme:
- A short press on ESC button will toggle ambiance between: No ambiance/relax/boost ambiance,  This setting will disappear after car is shut off.  (It is a shortcut for Icockpit amplify in NAC menu)
- A long press (>1sec) on ESC button will change theme on Cirocco (blue or bronze) without changing NAC theme. However when restarting the car, NAC will change his theme to match the cirocco theme.

After many try on my car I figured the following logic:
- Cirocco have 2 themes activated: 1=Blue and 2=Bronze
- NAC have 3 themes: 1=purple (AMETHYST), 2=red (RUBY), 3=yellow(GOLD)  (+unlisted NACblue theme)

When changing theme on NAC it also change Cirocco theme: 1purple-->1blue, 2red-->2bronze and 3Yellow-->2bronze.

When changing theme on Cirocco with ESC button, it will change NAC theme after restart: 1blue--> 1purple and 2bronze-->2red  (3yellow is not accessible)


If I try activating more themes in cirocco/NAC I have weird behaviour: NAC reboot to unlisted NACblue theme, and NAC theme selection menu disappear.cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)


Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it or try to make money from it)
