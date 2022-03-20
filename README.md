# DS4-cirocco-retrofit

This sketch will make the AAS (parking sensor) and SAM (blind spot monitoring) toogle key on NAC work (BSI telecoding for AAS is optional)

It also allow physical button and LED to work again. Check the connection yourself. Beware that LED are in +12v logic.

This skecth was tested on a DS4, and should also work on any C4 gen 2 (hatcback, sedan, etc)

HBA (High Beam Assist) do not work on DS4/C4 (car ignore CAN message from cluster), but it should be possible to make it work on DS5
Many thank to:
- Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration
- Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)


Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it or try to make money from it)
