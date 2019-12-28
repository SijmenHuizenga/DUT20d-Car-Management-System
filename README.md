# Car Management System

This tool tries to answer the question "what's going on with our Autonomaus System?". CMS
* connects to the car, collects all the health indicators and visualises them in a simple dashboard.
* offers you the ability to execute frequent tasks on the car like starting/stopping services, nodes and recordings.
* track's everything that is going on so you can see what happend when something went wrong.

# Architecture
The architecture of this system is as follows
```
Human <-> Webapp <-> Groundstation <-> Car
```
You are the human. When you use cms you open your webbrowser and enter the url of the groundstation.
A dashboard materializes on your screen. Using this webpage you can interact with the cms.
Later some rqt dashboard plugins might be provided. 
This webapp gets updates from and instructs the groundstation.
The groundstation talks in a few different ways to the car. 
It is a rosnode and subscribes to diagnostics topics, it executes actions on the car via ssh and checks availability using pings.
The groundstation runs on one (1) laptop where all users connect to.

## Developing
To develop you have to run two processes: the webapp and the groundstation.

For the **groundstation** you must first install the required python dependencies.
Because these packages are non-ros we must do this via pip. 
Run `pip install -r requirements.txt` from the current directory.
After that, the groundstation is just like any rosnode.
Just run `rosrun cms app.py` to run it.
_If you want to connect to an existing ros master (e.g. the car) you must first export environment variables ROS_MASTER_URI and ROS_IP in your shell_

For the **webapp** you must have the [npm](https://linuxize.com/post/how-to-install-node-js-on-ubuntu-18.04/) package manager installed.
_DO NOT RUN `apt-get install npm` because that will break ros!! Use the linked tutorial._
Run `npm install` in the current working directory to get all dependencies installed.
To run the app as a development server run `npm run start`.
This will launch a webserver that serves the compiled html/css/js files on the printed port. 
All requests to non-existing files will be forwarded to the groundstation of which it assumes runs on localhost.
 
## Deployment
Never done this, need to figure this out someday.

## Database
Groundstation stores all live data in a SQLite database. 
The database file is called `development.db` in current working directory. 
Later this name and location will be configurable. 

## The meaning of colors

| Color | Description |
| --- | --- |
| GREEN | Active: Something is actively in use. A process, service or node is running. A topic is actively processing messages. A network device is reachable. |
| GRAY | Idle: Something exists but not running. This color is only used for things that can exist without running. A systemd service is stopped, a topic exists but is not receiving messages |
| RED | Danger: Something is wrong and requires immediate action from a user. A process, service or node crashed. A topic that is being recorded is not sending any data.  |   
| ORANGE | Fault: CMS is unsure about the status. The last received update about something took place a long time ago, cms received unknown responses from ros, cms has a bug. | 
