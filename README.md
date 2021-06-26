![hal-kinematics](https://user-images.githubusercontent.com/44880102/123472425-29b60380-d5c5-11eb-8f47-a5ad096fb775.jpg)

# hal-kinematics
The kdl kinematics "c++" library used by a "c" hal component

You have to install the kdl kineatics c++ library on your system first.
https://github.com/grotius-cnc/orocos_kinematics_dynamics

The source code of this repository contains 2 qt projects that are opened together during coding. 
Parallel coding makes it eayser. You could also code in a text editor.

This source code is a basic example how to use the kdl kinematic library within a hal component.
+1 You could make any type of kinematic model with linuxcnc now, even when you are a dummy coder.


# The install workflow, tested with a linuxcnc "run in place" version.

- Compile the first qt project. 
```
  1. Open the first qt project "/next/next.pro"    
  2. "cntr+r" Run
  3. This will produce "libnext.so, libnext.1.so, libnext.1.0.so, libnext.so.1.0.0" 
  4. Stage one is now completed.
```

- Edit the Runtest file.
```
  1. open "runtest"
  2. edit line 7 (linuxcnc base dir), 8 (the dir where kinematic.so is produced)
```

- Compile the second qt project.
```
  1. open the second qt project "qt-test.creator"
  2. open the Makefile in a text editor (not in qt!). Edit your filepath at line 4 (points to halcompile executable), 28 (points to libnext)
  3. qt->projects->build->set build directory->your_path_to_git_clone
  4. qt->projects->run->set executable->your_path_to/runtest
  5. "cntr+r" Run
  6. This will produce "kinematic.so" and will run the app.
  7. stage two is completed.
```

# The usage workflow.

The runtest file has the workflow how to load the component.
It is preferred to use this file, or edit this file to your needs.

```
	Set up your machinemodel : machinemodel.hal
	Use the /doc/kuka pdf file as reference.
```

You can load the kinematic.so manually. Start linuxcnc and open up "show hal configuration"

```
  stop
	loadrt threads name1=base-thread fp1=0 period1=25000 name2=servo-thread period2=1000000
  loadrt kinematic
 	addf kinematic servo-thread
 	\-f your_path_to/machinemodel.hal
 	start
```

Now you can insert hal commands. For the hal pin reference file you can take a look at the "kinematic.comp" file.
```
  setp kinematic.cart-x 500
```

# Things to consider.

```
	1. The calculated cartesian tcp point is for example x600, y0, z550. 
	If you want to run a gcode, you have to create a extra set of xyz variables to offset
	the current tcp to your gcode. Or create a button that set's the current tcp offset to zero. (gcode starpoint).
	
	2. The gcode xyz output can be used as cartesian input. It will convert the xyz to joint values when using "ik_mode 1".
	
	3. The source code inverse kinematics (ik) is performed from machine init position.
	You can also expand to source code to use ik from current position. You have to create a extra variable for this.
	
	4. You could add a extra variable for the ammount of inverse kinematics iterations. They are now fixed at ~100.
	This could reduce the total program cyclus time. 
	
	5. When using the kinematic.so every servo-cycle, set the time interval=0.
	
	6. This source code is just an example of the kdl library that uses a kuka robot in this case.
	a more generic driver can be written. This is just a "kick off".
	If you want to write a generic machinemodel driver, you are welcome !.
```














