export const OUTDATED_AFTER_SECONDS = 5;

export interface Dashboard {
    rosnode :{
        up :boolean
    }
    logbook :LogbookLine[]
    ping :Ping
    ssh :SSH
    topics :{[key :string] :Topic}
    nodes :{[key :string] :Node}
    recording :Recording
    systemdservices: SystemdService[]
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
    lastseen: number
}

export interface Recording {
    is_recording :boolean
    filename :string
    bagfilename :string
    recordingduration :string
    selected_topics :string[]
    lastrefresh :number
}

export interface Node {
    lastseen :number
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