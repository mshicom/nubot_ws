nubot_standalone
================

Please follow the instructions below to make rtdb working

For the first time using rtdb
1. please modify the /etc/security/limits.conf file to deal with the error "setscheduler: operation not permitted":
 sudo gedit /etc/security/limits.conf
 put "* - rtprio 100" at the end
2. give the corresponding roobt ID as environment variable
 echo "AGENT=XXX" >> ~/.bashrc
3. restart your computer
 sudo reboot

After the first time
1. cd rtdb/bin
2. check if rtdb_parser is executable, if not, using the chmod command to make it executable, i.e., 
   chmod u+x ./rtdb_parser
3. type y when some language you do not know ask for y/n, usually type twice
4. ./rtdb_parser ../config/nubot.conf
5. cd ..
6. check if CreateIncludeLinks.sh is executable, if not, using chmod to make it executable, i.e., 
   chmod u+x CreateIncludeLinks.sh 
7. ./CreateIncludeLinks.sh
 

Now it is ready for compiling!
