/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

const char* helpText = "\n\
Key shortcuts \n\
    \n\
    H (shift + h): Print this help text to console \n\
    Space:         Toggle pause/play simulation \n\
    Return:        Step simulation (one timestep) then pause \n\
    TAB:           Move camera to next vehicle \n\
    Shift + TAB:   Move camera to previoius vehicle \n\
    Delete:        Same as above (Shift + TAB) \n\
    o:             Toggle show / hide OpenDRIVE road feature lines \n\
    O:             Toggle show / hide odr signal bounding boxes \n\
    u:             Toggle show / hide OSI road lines \n\
    y:             Toggle show / hide OSI road points \n\
    p:             Toggle show / hide environment 3D model \n\
    r:             Toggle show / hide sensor view frustums \n\
    R:             Toggle route waypoint visualization \n\
    i:             Toggle on-screen info text modes \n\
    j:             Toggle show trails after vehicles(4 modes: none / dots / lines / both) \n\
    n:             Toggle show active trajectories \n\
    , (comma):     Switch entity view : Model only / Bounding box / Model + Bounding box / None \n\
    K:             Print current camera position and orientation to console \n\
    ESC:           quit \n\
    \n\
    Arrow keys is used to drive externally controlled Ego vehicle: \n\
        Up:    Accelerate \n\
        Down:  Brake \n\
        Left:  Steer left \n\
        Right: Steer right \n\
    \n\
    1 - 9: Camera models acording to : \n\
        1: Custom camera model \n\
        2: Flight \n\
        3: Drive \n\
        4: Terrain \n\
        5: Orbit \n\
        6: FirstPerson \n\
        7: Spherical \n\
        8: NodeTracker \n\
        9: Trackball \n\
    \n\
    When custom camera model(1) is activated \n\
        k: Switch between the following sub models: \n\
           - Orbit        (camera facing vehicle, rotating around it) \n\
           - Fixed        (fix rotation, always straight behind vehicle) \n\
           - Flex         (imagine the camera attached to vehicle via an elastic string) \n\
           - Flex - orbit (Like flex but allows for roatation around vehicle) \n\
           - Top          (top view, fixed rotation, always straight above vehicle) \n\
           - Driver       (\"driver\" view, fixed at center of vehicle) \n\
    \n\
    Viewer options \n\
        f: Toggle full screen mode \n\
        t: Toggle textures \n\
        s: Rendering statistics \n\
        l: Toggle light \n\
        w: Toggle geometry mode(shading, wireframe, dots) \n\
        c: Save screenshot in JPEG format - in the folder where the application was started from \n\
        C: Toggle continuous screen capture (e.g for video creation) \n\
        h: Help \n\
    \n\
Mouse control \n\
    \n\
    Left:   Rotate \n\
    Right:  Zoom \n\
    Middle: Pan \n\
    \n\
    This is typical, exact behaviour depends on active camera model. \n\
";
