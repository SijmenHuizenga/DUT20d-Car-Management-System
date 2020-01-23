# This message describes how the recording is configured.
# All recording parameters are stored in /var/rosrecord.env.
# This message is published on topic /cms/recording/config every 3 seconds or when the /var/rosrecord.env file is changed.

# The prefix of the bag filename that new recordings will have. This is without .bag(.active) suffix.
string filename

# The topics selected for recording.
string[] topics