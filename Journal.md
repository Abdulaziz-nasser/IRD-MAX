# **Troubleshooting**

During the development of the vehicle, we ran into a problem while setting up and testing the system. This page shows what happened, what we tried, and how we fixed it.

## **Problem 1 — [Faulty usb drive]** 
<img width="640" height="480" alt="pic_error1" src="https://github.com/user-attachments/assets/a6a34fac-dce3-4bc8-bbc6-130319bf3a2c" />


### **Problem**

During installation, we found a problem with installing the software. 

### **What we tried**

* We Tried commands.
* we waited for 30 mins for the machine
* recheck the usb

### **Solution**

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
The connection worked, allowing us to access the Jetson desktop from the laptop.

