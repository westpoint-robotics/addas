# ADDAS

### Setting up Pixhawk to work on Nano TALON UAS
1. Get imaged Linux/ROS computer from EECS department.
2. Login.
3. Check hosts file is consistent with host name:
    * `echo $HOSTNAME` 
    * Copy the output (ex: ros305)
    * `gksu gedit /etc/hosts` 
    * Make sure the line 2 of the file is the same as $HOSTNAME.  If not, then correct/past in, and save.
4. Connect to the internet (EECS DS3 Network).  Preferably select a high speed wired line.  EECSNet instructions forthcoming.
5. Install Git:
    * `sudo apt-get install git`
    * NOTE: may have to restart computer if error.
6. Create SSH key:
    * `ssh-keygen -t rsa -C "your.email@example.com" -b 4096`  
    * Hit enter through all questions to keep defaults path, and no password.
7. Install xclip:
    * `sudo apt-get install xclip`
8. Copy SSH key: 
    * `xclip -sel clip < ~/.ssh/id_rsa.pub`
9. Log into: https://gitlab.nps.edu
    * Go to Profile Settings > SSH Keys tab.  Paste in ssh key (starts with ssh-rsa, end with email). Set title to computer name.
10. Clone repo:
    * `cd $HOME`
    * `git clone git@gitlab.nps.edu:sasc/sasc-help.git`
11. Install SASC by typing following commands (will have to enter password along the way): 
    * `sudo apt-get update`
    * `sudo apt-get upgrade`
    * `cd $HOME/sasc-help`
    * `./sasc_installer.py -I –s`
    * Once the installation is complete, restart the computer
12. Initialize 10 planes after installation complete    
    * `init_n_sitls.bash 10`
    * If an error occurs, check the last lines of the bashrc file (`sudo gedit .bashrc`) against a working machine. 

### Other useful info (TODO: replace this with other sections)

