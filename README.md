# PixelPanelWand
Wand IMU code for the PixelPanel controller

### PixelPanelWand:  ESP32 code written in the Arduino PlatformIO IDE.

    PixelPanelWand is the code that controls the hand-held wand that is used to control
    the PixelPanel.  It has a 9-axis IMU (MPU9250) to measure the user's movements.  We 
    send ESP-NOW packets to the PixelPanel to control the scenes and PixelPanel settings.

    The wand operates in two modes:  Pose-mode and gesture-mode.  In pose-mode, information
    about the location and attitude of the wand is transmitted to control effects in the 
    currently displayed scene.  In gesture-mode the wand transmitts decoded gesture-commands
    (swipe-up/down/left/right, thrust-in/out, circle-cw/ccw, etc.).  Thes commands are used
    my the PixelPanel to execute control options (change scene, change brightness, change 
    diffuser screen distance, etc.

    Here are the wand coordinate definitions used in the code:

    WAND COORDINATE / SIGN CONVENTION

    Translation:
      +X = forward / toward wand tip
      -X = backward / toward handle

      +Y = right
      -Y = left

      +Z = down
      -Z = up

    Rotation:
      +Roll  = clockwise/right
      -Roll  = counterclockwise/left

      +Pitch = front/tip up
      -Pitch = front/tip down

      +Yaw   = clockwise viewed from above
      -Yaw   = counterclockwise viewed from above

    The wand has a single momentary-contact button used to switch between modes.  Also, 
    on boot, if the button is held vor ~2.5 seconds we will enter a calibration routine 
    where the wand is oriented around all six directions (+/-X, +/-Y, +/-Z) and lying still
    to calibrate the accelerometer and gyro (and eventually the magnetometer).  The offset
    bias and scale factors are stored in non-volatile RAM using ESP Preferences.

    I am collaborating with chatGPT in designing and coding this project.
