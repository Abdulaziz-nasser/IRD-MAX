# **Troubleshooting**

During the development of the vehicle, we ran into a problem while setting up and testing the system. This page shows what happened, what we tried, and how we fixed it.

## **Problem 1 — [Faulty usb drive]** 
<img width="640" height="480" alt="maybe_a_problem" src="https://github.com/user-attachments/assets/881410ae-a910-4c82-b6fe-b376b6668296" />



### **Problem**

During installation, we found a problem with installing the software. 

### **What we tried**

* We Tried commands.
* we waited for 30 mins for the machine
* recheck the usb

### **Solution**
<img width="640" height="480" alt="worked" src="https://github.com/user-attachments/assets/e0a43b2a-5aad-4f6c-968b-c6432be90d0d" />


We gotten a another usb drive to reinstall jetpack software.

### **Result**

After making the change, we tested the system again and it worked correctly. 
<img width="640" height="480" alt="pic_solve1" src="https://github.com/user-attachments/assets/f72a838c-ae70-4830-945c-4e5ff570c717" />

## **Problem 2 — NoMachine Connection** 
<img width="640" height="480" alt="photo_no_machine_not_done_#crying" src="https://github.com/user-attachments/assets/20470a50-e19c-4336-aa9e-58a59ea4540b" />


### **Problem**

The laptop had NoMachine 7.8 and the Jetson had version 10. We could not connect, and NoMachine showed “The connection with the server was lost.”

### **What We Tried**

- Tried connecting with the existing versions.
- Tried downgrading the Jetson to version 9.8, but the connection still failed.

### **Solution**

We downloaded and installed the same NoMachine version on both devices.

### **Result**
<img width="640" height="480" alt="photo_no_machine_done" src="https://github.com/user-attachments/assets/07575dfe-7652-491f-a7ad-9f5fa959bb84" />

The connection worked allowing us to access the jetson desktop from the laptop easily ( finally :-:)

## **Problem 3 — Yaw and Straight Driving**

### **Problem**

During track testing, the car kept steering left and right when it should have been driving straight.

### **What We Tried**

* Tested straight driving at low speed.
* Adjusted the yaw filter and steering limits.
* Checked the yaw readings and steering commands. We found the code recalculated corrections faster than fresh yaw readings arrived and rounded steering to whole degrees, which could cause uneven corrections.

### **Solution**

We added `[STRAIGHT]` to hold one target heading on each straight section. It uses fresh yaw readings, makes small, smooth corrections, and resets its correction history after turns.

### **Result**

We made two versions with the same speeds and turns to compare them:

**Without the yaw system:** [slow_without_yaw.py](software/jetson/slow_without_yaw.py) keeps steering centered between turns.


https://github.com/user-attachments/assets/1d59722b-05b5-4f70-8cae-8201ca815710





**With the yaw system:** [slow_with_yaw.py](software/jetson/slow_with_yaw.py) uses `[STRAIGHT]` to correct the heading between turns.




https://github.com/user-attachments/assets/0a3b3312-4361-43b0-a533-556fb77a1832



Both versions still use the IMU for corner turns.
