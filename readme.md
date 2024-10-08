**Balance Bot System**
======================

**License**
----------

This project is licensed under the **GNU General Public License (GPL) version 2**.

**Features**
------------

*   **Automated Balance Management**: The balance bot system automatically tracks and updates robot real-time.
*   **Cheap components!**: Using the MPU6050 and a RaspberryPi 3, 
*   **Interfaces to hoverboard systems** 


**Technical Details**
-------------------

*   **Programming Language**: C++

**Installation and Setup**
-------------------------

1.  Clone this repository: `git clone https://github.com/your-username/balance-bot-system.git`
2.  Modify PID.conf to have good starting parameters
3.  make release or make debug


**Running the Application**
---------------------------

1.  sudo release/balanceSam or sudo debug/balanceSam
2.  In another terminal edit PID.conf and the code will automatically notice the file change and load the new values.

**Troubleshooting**
------------------

*   Use the debug/balanceSam to see values. 

**Contributing**
---------------

*   Fork this repository
*   Make changes to the code
*   Commit changes with descriptive commit messages
*   Create a new pull request

