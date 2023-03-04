# ROB515_INTRO2

# 0. Required Packages

```console
pip3 install logdecorator
```

# 1. Running the program

## 1.1 Shell the picar-x
Open one terminal and secure shell (ssh) the picar-x from your host pc.

Choice 1: If you are within OSU campus (ssh user@PiCar#.engr.oregonstate.edu)
```console
ssh alejo@PiCar19.engr.oregonstate.edu 
```

Choice 2: If you are in another wireless network (ssh user@piname.local)
```console
ssh alejo@alejopi.local
```
## 1.2 Update from Github
Terminal 1: Keep one terminal opened at the repository directory to be able to pull the updates when needed.
```console
git pull
```

## 1.3. Run your code
Terminal 2: cd the directory where you keep the code
```console
cd ~/RobotSystems/picar-x/picarx/
```
and then run the script
```console
sudo python3 velasale_week2.py
```

You could also run the calibration script:
```console
sudo python3 ~/RobotSystems/picar-x/example/calibration/calibration.py
```