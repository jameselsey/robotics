# Hardware
The hardware is evolving as I add new features and improvements, I wanted to make it as modular as possible so I can add new layers and move modules around.

## Chassis
The chassis is a Xiaor Geek tank model, these are easy to find on Amazon or AliExpress for around AUD$120, it will come with 2 12v DC motors.

I had to make a few modifications to this, I needed to extend the front bay so that a Ryobi battery could fit, I used some scrap PVC pipe flattened with a heat gun to make some extension brackets.

I also took the cross plate that was towards the back, and moved it forward so there is enough room for a battery to sit safely.

![Chassis](/img/r2-battery-bay.jpg)

Lastly, the holes in the top plate of the chassis weren't in the best place, so I drilled 4 new holes, each near the corners to maximise space for modules. I used those same holes on all layers of the stack for the spacers to sit.

## Power
I wanted to run everything from a Ryobi 18v power tool battery, because I already had one spare, and a charger, and considering they take a lot of abuse on work sites I thought they'd be tough enough and perhaps not as volatile as a regular RC lipo. I also wanted to keep the costs down as batteries, a balance charger, and battery boxes here would have cost me $500.

As you can see from the under side of the robot, I fixed a clear acrylic sheet with short spacers, and was then able to attach 2 buck converters. Both are connected to the battery, one bucks down to 12v, the other to 5v.

I have a smaller buck converter on the lower layer that bucks the 5v down to 3.3v, which I use for the encoders and LED lights.

![Buck Converters](/img/r2-under.jpg)

![3v Buck Converter](/img/r2-back-encoders.jpg)

## Sensors
The lidar sensor of course needs to be placed on top to get 360 degree scanning, I could have placed lower down but then I would not get full scanning.

![Top Layer](/img/r2-top-layer.jpg)

The bracket for the webcam I made from a piece of PVC pipe and a heat gun, I softened it up, flattened with some steel, then folded the bracket around the webcam so it slot in tightly. I needed to place the webcam up high so it could see over the battery.

Below the webcam is a USB speaker, like with the webcam, it didn't have any good fixing points so I made a bracket from PVC pipe.

## Electronics
I started by using breadboards and jumper cables, but it quickly became a spaghetti mess. Once I had a better soldering iron, I experimented with some JST-XH connectors and a perfboard, this made it much easier to disconnect and swap in new modules if I needed to replace a motor.

![Electronics](/img/r2-back.jpg)