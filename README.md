# Internet of Things Programming Projects, 2nd Edition

<a href="https://www.packtpub.com/product/internet-of-things-programming-projects-second-edition/9781835082959"><img src="https://content.packt.com/B21282/cover_image_small.jpg" alt="no-image" height="256px" align="right"></a>

This is the code repository for [Internet of Things Programming Projects, 2nd Edition](https://www.packtpub.com/product/internet-of-things-programming-projects-second-edition/9781835082959), published by Packt.

**Build exciting IoT projects using Raspberry Pi 5, Raspberry Pi Pico, and Python**

## What is this book about?
This second edition of Internet of Things Programming Projects will equip both beginners and experienced programmers with the knowledge and skills needed to create projects using Raspberry Pi, web services, LoRa, wireless communication, and MQTT.

This book covers the following exciting features:
* Integrate web services into projects for real-time data display and analysis
* Integrate sensors, motors, and displays to build smart IoT devices
* Build a weather indicator using servo motors and LEDs
* Create an autonomous IoT robot car capable of performing tasks
* Develop a home security system with real-time alerts and SMS notifications
* Explore LoRa and LoRaWAN for remote environmental monitoring

If you feel this book is for you, get your [copy](https://www.amazon.com/Internet-Things-Programming-Projects-Raspberry-ebook/dp/B0CW1MLT83) today!

<a href="https://www.packtpub.com/?utm_source=github&utm_medium=banner&utm_campaign=GitHubBanner"><img src="https://raw.githubusercontent.com/PacktPublishing/GitHub/master/GitHub.png" 
alt="https://www.packtpub.com/" border="5" /></a>

## Instructions and Navigations
All of the code is organized into folders. For example, Chapter1.

The code will look like the following:
```
def timer_callback(self):
    if self.mqtt_message.should_draw_circle:
        self.vel_msg.linear.x = 1.0
        self.vel_msg.angular.z = 1.0
    else:
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
    self.publisher.publish(self.vel_msg)
```

**Following is what you need for this book:**
This book is for beginners as well as experienced programmers, IoT developers, and Raspberry Pi enthusiasts. With just basic knowledge of IoT, you can dive right in and explore the projects with ease.

With the following software and hardware list you can run all code files present in the book (Chapter 1-14).
### Software and Hardware List
| Software/hardware covered in the book | OS required |
| ------------------------------------ | ----------------------------------- |
| Python | Raspberry Pi OS and Ubuntu Linux |
| CircuitPython |   |
| MQTT |   |
| OpenCV |   |
| LoRa |   |
| ROS |   |
| Raspberry Pi |   |
| Raspberry Pi Pico |   |
| Raspberry Pi Sense HAT |   |
| Various sensors (PIR and temperature) |   |
| M5Stack ATOM Matrix |   |
| Pico robotics board |   |



### Related products
* Architectural Patterns and Techniques for Developing IoT Solutions [[Packt]](https://www.packtpub.com/product/architectural-patterns-and-techniques-for-developing-iot-solutions/9781803245492) [[Amazon]](https://www.amazon.com/dp/1803245492)

* Practical Python Programming for IoT [[Packt]](https://www.packtpub.com/product/practical-python-programming-for-iot/9781838982461) [[Amazon]](https://www.amazon.com/dp/1838982469)


## Get to Know the Author
Colin Dow has been 3D printing since 2013, starting with the laser-cut wooden frame version of the Ultimaker 3D printer. Over the years, he has used a dozen or so 3D printers, including MakerBots, PrintrBots, early Prusa i3s, delta printers, and liquid resin printers. Colin has been working with OpenSCAD since 2014, using it to design and manufacture model rocketry parts for his model rocketry business. Through his aerospace workshops, he has introduced many students to 3D printing, including in-class demonstrations. In recent years, Colin has been designing and building automated drones for his drone startup using 3D printers and OpenSCAD.


## Other books by the author
* Simplifying 3D Printing with OpenSCAD [[Packt]](https://www.packtpub.com/product/simplifying-3d-printing-with-openscad/9781801813174) [[Amazon]](https://www.amazon.com/Simplifying-3D-Printing-OpenSCAD-programs/dp/1801813175)
* Hands-On Edge Analytics with Azure IoT [[Packt]](https://www.packtpub.com/product/hands-on-edge-analytics-with-azure-iot/9781838829902) [[Amazon]](https://www.amazon.com/Hands-Edge-Analytics-Azure-IoT/dp/1838829903)
* Mastering IOT [[Packt]](https://www.packtpub.com/product/mastering-iot/9781838645434) [[Amazon]](https://www.amazon.com/Mastering-IOT-solutions-monitor-infrastructure/dp/1838645438)
* Internet of Things Programming Projects [[Packt]](https://www.packtpub.com/product/internet-of-things-programming-projects/9781789134803) [[Amazon]](https://www.amazon.com/Internet-Things-Programming-Projects-solutions/dp/1789134803)
