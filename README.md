# ViewFindAR_MagicLeap_v0.22
ViewFindAR build for Magic Leap One AR Headset with Lumin SDK 0.22

## What is ViewFindAR?
------
ViewFindAR is a research project that attempts to use a combination of AR and CV algorithms in order to take a 2D image
of an object and re-display it from an arbitrary angle, effectively creating a virtual camera. ViewFindAR does this by 
generalizing the object of focus as a cuboid, placing a 3D bounding box around the object and storing the faces of 
the cuboid as textures. 
These rectified textures are then warped to re-projected coordinates that correspond to the same points but from a different camera angle using classical CV Homography warping. 
Combining these warped textures results in an image that closely resembles the object from a separate angle. In order to address the parallax and lighting discrepancies that arise when shoving a cylinder/sphere into a cuboid-sized hole, multiple textures of the same face are stored, from different angles. 

## Using ViewFindAR
------
For the Magic Leap build of ViewFindAR, a Magic Leap One headset is required. The scene [ViewFindAR.unity] should be opened 
in a 2019.2 build of Unity, and then built onto a Magic Leap Headset with Lumin SDK 0.22. 

The Magic Leap One build of Unity currently does not have automatic corner detection, so corners of the object must be placed manually via Raycast. A green circle cursor should appear as a guide to where the raycast is landing, and pressing the home button will spawn a purple world-marker. The world-markers must be placed in a specific order: 
```
   0-------1
 / |     / |
3--+----6  |
|  .----|--2
| /     | /
4-------5
```
This process is automated in the Android build with ArUco markers, but while that feature is worked on in the ML build, this is the process we have in the meantime. 

Once the 7 world-markers have been instantiated, press the home button again to toggle recording. A red [REC] indicator should be visible in the top-left corner of the HUD. Once recording is enabled, press the bumper on the controller to capture an image. Make sure you get the entirety of the object of focus in the frame, and keep in mind the Magic Leap One camera used for capture is on the left side of the device's frame. Every capture is cached in order to reconcile the display from different perspectives. Recording can be disabled with another press of the home button, in order to marginally increase frame rate, and capture demonstration videos with the ```mldb capture video -w filename.mp4``` command. 
