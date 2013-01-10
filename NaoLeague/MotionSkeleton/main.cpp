/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <iostream>
#include <string>

// include the proxies you'll be using
#include <alproxies/almotionproxy.h>

#define PORT 9559

using namespace std;

int main(int argc, char const *argv[])
{
    // quick and dirty command parsing to get the IP
    string ip = "10.0.0.50";//string( argv[1] );

    // create a proxy, specifying the IP and port (port is always 9559)
    AL::ALMotionProxy motProxy(ip, PORT);

    // make sure the entire robot is stiff
    motProxy.setStiffnesses("Body", 0.8);

    while (true) {
        motProxy.setAngles("RShoulderRoll", 0, 0.7);
        motProxy.setAngles("RShoulderRoll", 1, 0.7);
    }

    // remove stiffness
    motProxy.setStiffnesses("Body", 0);

    return 0;
}
