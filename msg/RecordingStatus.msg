# This message is published every second to /cms/recording/statusmsg.
# In this message you find information about the status of the recording.
# All information is collected by querying the filesystem and reading the status of the rosrecord systemctl service.

# Is the recording active?
bool recordingactive

# The name of the active bag, including .bag.active suffix
string filename

# The topics that rosrecord subscribed to, according to rosrecord
string[] topics

# The duration of the recording, measured by inspecting the timestamp when rosrecord was started in seconds.
uint16 duration

# The filesize of the active bag in megabites.
uint16 filesize