#!/bin/bash
echo "Restarting roboRIO user code in debug mode..."
#ssh -l lvuser roborio-1736-frc.local . /etc/profile.d/natinst-path.sh
ssh -l lvuser roborio-1736-frc.local /usr/local/frc/bin/frcKillRobot.sh
ssh -l lvuser roborio-1736-frc.local  /usr/local/frc/bin/netconsole-host /usr/local/frc/JRE/bin/java -Djava.library.path=/usr/local/frc/lib/ -XX:+UsePerfData -agentlib:jdwp=transport=dt_socket,address=8348,server=y,suspend=y -jar /home/lvuser/FRCUserProgram.jar



echo "Finished! You may now launch the eclipse debug target."