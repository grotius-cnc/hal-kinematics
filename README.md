![hal-kinematics](https://user-images.githubusercontent.com/44880102/123472425-29b60380-d5c5-11eb-8f47-a5ad096fb775.jpg)

# hal-kinematics
The kdl kinematics "c++" library used by a "c" hal component

You have to install the kdl kineatics c++ library on your system first.
https://github.com/grotius-cnc/orocos_kinematics_dynamics

The source code of this repository contains 2 qt projects that are opened together during coding. 
Parallel coding makes it eayser. You could also code in a text editor.

This source code is a basic example how to use the kdl kinematic library within a hal component.
+1 You could make any type of kinematic model with linuxcnc now, even when you are a dummy coder.


The install workflow, tested with a run in place linuxcnc version :

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

The usage workflow.

The runtest file has the workflow how to load the component.
It is preferred to use this file, or edit this file to your needs.

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
















