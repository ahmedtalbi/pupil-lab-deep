# Pupil
Open source eye tracking software platform that started as a thesis project at MIT. Pupil is a project in active, community driven development. Pupil mobile eye tracking hardware is accessible, hackable, and affordable. The software is open source and written in `Python` and `C++` when speed is an issue.

Our vision is to create tools for a diverse group of people interested in learning about eye tracking and conducting their eye tracking projects.

[![Gitter](https://badges.gitter.im/pupil-labs/pupil.svg)](https://gitter.im/pupil-labs/pupil?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)

## Project Website
For an intro to the Pupil mobile eye tracking platform have a look at the [Pupil Labs Website](http://pupil-labs.com "Pupil Labs").


## License
All source code written by Pupil Labs is open for use in compliance with the [GNU Lesser General Public License (LGPL v3.0)](http://www.gnu.org/licenses/lgpl-3.0.en.html). We want you to change and improve the code -- make a fork! Make sure to share your work with the community! See the wiki for more info on the [license](https://github.com/pupil-labs/pupil/wiki/License "License"). For support and custom licencing [contact us!](https://github.com/pupil-labs/pupil/wiki/Community#email "email us")

## Adding a Object detection Module based on SSD.

In the context of a practical course offered by the ICS chair of the University of Munich, an object detection module has been added to the pupil framework.

### Citing SSD

Please cite SSD in your publications if it helps your research:

    @inproceedings{liu2016ssd,
      title = {{SSD}: Single Shot MultiBox Detector},
      author = {Liu, Wei and Anguelov, Dragomir and Erhan, Dumitru and Szegedy, Christian and Reed, Scott and Fu, Cheng-Yang and Berg, Alexander C.},
      booktitle = {ECCV},
      year = {2016}
    }

## Setup:

These installation instructions are tested using Ubuntu 16.04 or higher running on many machines. Do not run Pupil on a VM unless you know what you are doing.

1) The requirements for the installation of the pupil framework in Linux are listed in this [link](https://github.com/pupil-labs/pupil/wiki/Dependencies-Installation-Linux)

2) Install the SSD unified framework for object detection: [SSD](https://github.com/weiliu89/caffe/tree/ssd)

3) Add the caffe framework to the python env path: add this line to your ~/.bashrc: 
export PYTHONPATH="${PYTHONPATH}:/home/ahmed/development/SSD/caffe/python"

## Added work
in this project we implemented three modules: this modules have been added to the framework as plugins. To call any of these plugins the user needs to call the $eye-tracker_Root/pupil_src/capture/main.py by calling them from the side bar.
### Template matching module:

A template matching plugin based on opencv. You can choose between 6 template matching methodes.
In order to use this function, adjust your root folder in the template matching function(line 85) and then just enter the name of one of the templates stored in $eye-tracker_Root/templates.
If you want to try out a new template just add it to the previously mentioned folder.
Finally, pressing the save image button allows saving the images to the $eye-tracker_Root/SavedImages folder.

### ROS Publisher:

A roscore and a rosnode publishing three topics: 
/Frame: the frames received by the camera
/GazePos: normalized position of the gaze, their confidence and their timestamp
/Frame_timestamp: the timestamp of the frame (for comparison purposes with the gaze positions)

### Object Detection with SSD.
A detection Plugin based on SSD (Single shot multibox detector)
This Plugin is using the gaze position from the pupil headset, in order to localize objects based on their location. You can control the size of the Region of Interest (ROI) by changing the ratio. 
You can also use the webcam instead of the pupil headset.
Controlling the detector is done by pressing the OD button on the screen or the detector button on the side.

####remark: in line 203 if the detection is too slow using the headset, take this line out of the for loop. However using a suficient GPU power this shouldn't be an issue.


