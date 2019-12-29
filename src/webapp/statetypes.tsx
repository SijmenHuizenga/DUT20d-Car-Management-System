export const OUTDATED_AFTER_SECONDS = 5;

export interface Dashboard {
    rosnode :{
        up :boolean
    }
    logbook :LogbookLine[]
    ping :Ping
    ssh :SSH
    recording :Recording
    systemdservices: SystemdService[]
    topics :Topic[]
    nodes :Node[]
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
    topicstatistics :TopicStatistic[]
}

export interface Ping {
    timestamp :number
    success :boolean
}

export interface LogbookLine {
    rowid :number
    timestamp :number
    text :string
    source :string
}

export interface Topic {
    name :string
    lastseen: number
}

export interface TopicType {
    topicname :string
    messagetype: string
    lastseen :number
}

export interface Node {
    name :string
    lastseen :number
}

export interface TopicPublication {
    nodename :string
    topicname :string
    lastseen :string
}

export interface TopicSubscription {
    nodename :string
    topicname :string
    lastseen :string
}

export interface TopicStatistic {
    node_sub :string
    node_pub :string
    window_start :number
    window_stop :number
    delivered_msgs :number
    dropped_msgs :number
    traffic :number
    lastseen :number
}

export interface Recording {
    is_recording :boolean
    filename :string
    bagfilename :string
    recordingduration :string
    selected_topics :string[]
    lastrefresh :number
}

export interface SSH {
    connected: boolean
    lastping: number
    uptime: string
}

export enum SystemdServiceRunning {
    running = "running",
    stopped = "stopped",
    error = "error"
}

export enum SystemdServiceEnabled {
    enabled = "enabled",
    disabled = "disabled",
    error = "error"
}

export interface SystemdService {
    name :string
    running :SystemdServiceRunning
    enabled :SystemdServiceEnabled
    statustext :string
    lastupdate :number
}