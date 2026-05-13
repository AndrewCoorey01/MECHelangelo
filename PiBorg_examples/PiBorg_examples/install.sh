#!/bin/bash

DTB=`pwd`

echo '=== Make scripts executable ==='
chmod a+x *.py
chmod a+x *.sh

echo '=== Create a desktop shortcut for the GUI example ==='
TB_SHORTCUT="${HOME}/Desktop/ThunderBorg.desktop"
echo "[Desktop Entry]" > ${TB_SHORTCUT}
echo "Encoding=UTF-8" >> ${TB_SHORTCUT}
echo "Version=1.0" >> ${TB_SHORTCUT}
echo "Type=Application" >> ${TB_SHORTCUT}
echo "Exec=${DTB}/tbGui.py" >> ${TB_SHORTCUT}
echo "Icon=${DTB}/piborg.ico" >> ${TB_SHORTCUT}
echo "Terminal=false" >> ${TB_SHORTCUT}
echo "Name=ThunderBorg Demo GUI" >> ${TB_SHORTCUT}
echo "Comment=ThunderBorg demonstration GUI" >> ${TB_SHORTCUT}
echo "Categories=Application;Development;" >> ${TB_SHORTCUT}

echo '=== Create a desktop shortcut for the LED GUI example ==='
TB_SHORTCUT="${HOME}/Desktop/ThunderBorg-led.desktop"
echo "[Desktop Entry]" > ${TB_SHORTCUT}
echo "Encoding=UTF-8" >> ${TB_SHORTCUT}
echo "Version=1.0" >> ${TB_SHORTCUT}
echo "Type=Application" >> ${TB_SHORTCUT}
echo "Exec=${DTB}/tbLedGui.py" >> ${TB_SHORTCUT}
echo "Icon=${DTB}/piborg.ico" >> ${TB_SHORTCUT}
echo "Terminal=false" >> ${TB_SHORTCUT}
echo "Name=ThunderBorg LED Demo GUI" >> ${TB_SHORTCUT}
echo "Comment=ThunderBorg LED demonstration GUI" >> ${TB_SHORTCUT}
echo "Categories=Application;Development;" >> ${TB_SHORTCUT}

echo '=== Finished ==='
echo ''
echo 'Your Raspberry Pi should now be setup for running ThunderBorg'
echo 'Please restart your Raspberry Pi and ensure the I2C interface is enabled'
