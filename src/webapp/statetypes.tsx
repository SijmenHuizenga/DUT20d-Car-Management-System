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
    systemdservices: {[key :string]: SystemdService}
}

export interface Ping {
    timestamp :number
    success :boolean
}

export interface LogbookLine {
    rowid :number
    timestamp :number
    text :string
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
}

export interface Node {
    lastseen :number
}

export interface SSH {
    connected: boolean
    lastping: number
    uptime: string
}

export interface SystemdService {
    statustext :string
    status :string
    lastupdate :number
    enabled :boolean
}