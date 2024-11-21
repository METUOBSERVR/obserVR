## HOW TO RUN
1. Update apt get with "sudo apt update" (Linux)
2. Upgrade apt get with "sudo apt upgrade" (Linux)
3. Get bcm2835 package with "wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.xx.tar.gz" (current directory so do not download it into porject directory)
4. Unpackage bcm2835 with "tar zxvf bcm2835-1.xx.tar.gz" (Check lastest version)
5. Go to bcm2835 directory with "cd bcm2835-1.xx"
6. Run the configuration with "./configure"
7. Run make file with "make"
8. Run make install with "sudo make install"
9. Compile code and check whether it is install correctly or not 
10. Make the output file executable with "chmod +x -file name-"



## RUN MAKE FILE
-   "mingw32-make" for windows   
-   "make" for linux