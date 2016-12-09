cp ArduCopter/revomini_MP32V1F4.bin Release
cp ArduCopter/revomini_MP32V1F4.hex Release

zip -r latest.zip Release
git add . -A
