# DS4-cirocco-retrofit

New updated sketch see Cirocco_retrofit\Cirocco_retrofit_speed_afil.ino:



The function of this sktech:

- Make AAS (Aide Au Stationement, parking sensor) and SAM (Surveillance Angles Mort, blind spots monitor) NAC toogle switch work

- Speed limit display from CVM and NAV data (see comment)

- Fix SAM light on cirocco (frame 0x321)  (Thanks to Infizer)

- Fix animation on cirocco (frame 0x236)  (Thanks to Infizer)

- Display green/orange line on cirrocco (AFIL and LKA need to be activated in cirocco) (Thanks to Infizer)

- Make AFIL indicator flash when lines are being crossed without turn indicator (see comment)(Thanks to Infizer)


  


 For speed limit display it follow this logic:

- show end of speedlimit sign for 3s when it is read by CVM

- show CVM speed when it is reliable (red sign)

- keep showing CVM speed in red sign after CVM loose reliability. Only if NAV speed have the same value and has not changed since it lost CVM reliability (CVM reliability is easily lost so I "extend" it)

- show NAV speed (grey sign)

- if no nav speed is avilable it display last read sign (grey sign and remove 1km/h. Example last sign=70km/h display 69 in grey sign)

- if no previous sign (and no nav speed) it display nothing


For AFIL  (Alerte Franchissement Involontaire de Ligne, lign dectection):

- display green orange line as soon as they are detected at any time (require LKA activated in cirocco)

- Flash AFIL light when one line become orange and if one(and not 2) turning indicator is ON  and when speed is over 50km/h (require AFIL activated in cirocco)

- When left/right lines are both detected: margin calculation for AFIL threeshold (large road= big margin small road= small margin) default value is 1300.






This sketch will make the AAS (parking sensor) and SAM (blind spot monitoring) toogle key on NAC work (BSI telecoding for AAS is optional)

It use same hardware as arduino CanHacker

It also allow physical button and LED to work again. Check the connection yourself. Beware that LED are in +12v logic.

This skecth was tested on a DS4, and should also work on any C4 gen 2 (hatcback, sedan, etc)

HBA (High Beam Assist) do not work on DS4/C4 (car ignore CAN message from cluster), but it should be possible to make it work on DS5


Many thanks to:
- Infizer for many items  (https://github.com/infizer91/can_extender  , https://www.drive2.ru/users/infizer/ , https://telecoder.ru/index.php )
- Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration
- Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)


Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it or try to make money from it)
