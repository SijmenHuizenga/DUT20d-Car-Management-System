# Car Management System

This tool tries to answer the question "what's going on with our Autonomaus System?". CMS
* connects to the car, collects all the health indicators and visualises them in a simple dashboard.
* offers you the ability to execute frequent tasks on the car like starting/stopping services, nodes and recordings.

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

## Running CMS
For the **groundstation** you must first install the required python dependencies.
Because these packages are non-ros we must do this via pip. 
Run `pip install -r requirements.txt` from the current directory.

For the **webapp** you must have the npm package manager installed. 
You can install it using the the [cms](/tooling/setup/scripts/cms.sh) installscript
_DO NOT RUN `apt-get install npm` because that will break ros!!_
Run `npm install --dev` in the current working directory to get all dependencies installed.
Run `npm run build`, wait for it to finish

Now, create a new file `/var/cms-keys.yaml` and add the keys. What keys? Ask Sijmen.

Create the directory `/var/cms/` and give it permissions to the current users to create files. 
CMS will store the logbook databases in there.

After that, the groundstation can be started using `groundstation.launch`.
Just run `roslaunch cms groundstation.launch` to run it.
_If you want to connect to an existing ros master (e.g. the car) you must first export environment variables ROS_MASTER_URI and ROS_IP in your shell_

To run the app as a development server run `npm run start`.
This will launch a webserver that serves the compiled html/css/js files on the printed port. 
All requests to non-existing files will be forwarded to the groundstation of which it assumes runs on localhost.
 
So go to `localhost:1097` or `<ip>:1097` to view cms!!
 
## Deployment

On the car you can optionally have a few extra nodes running.
The statistics node can be launched using [statistics.launch](/src/car/launch/statistics.launch).
The recording management node is included in [infrastructure.launch](/src/car/launch/infrastructure.launch).

## Database
Groundstation stores all live data in a SQLite database. 
The database file is called `logbook-<date>.db` in this directory. 

## The meaning of colors

| Color | Description |
| --- | --- |
| GREEN | Active: Something is actively in use. A process, service or node is running. A topic is actively processing messages. A network device is reachable. |
| GRAY | Idle: Something exists but not running. This color is only used for things that can exist without running. A systemd service is stopped, a topic exists but is not receiving messages |
| RED | Danger: Something is wrong and requires immediate action from a user. A process, service or node crashed. A topic that is being recorded is not sending any data.  |   
| ORANGE | Fault: CMS is unsure about the status. The last received update about something took place a long time ago, cms received unknown responses from ros, cms has a bug. | 
