cp ArduCopter/revomini_MP32V1F4.bin Release/Copter
cp ArduCopter/revomini_MP32V1F4.hex Release/Copter

zip -r latest.zip Release
git add . -A
