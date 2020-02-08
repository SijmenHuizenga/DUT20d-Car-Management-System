import {IndicatorColor, isRecent} from "./util/Indicator";

export const OUTDATED_AFTER_SECONDS = 5;

export interface Dashboard {
    rosnode :{
        up :boolean
    }
    ping :Ping
    ssh :SSH
    recording :Recording
    systemdservices: SystemdService[]
    topics :Topic[]
    nodes :Node[]
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
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
    statistics: TopicStatistic | null
}

export function getTopicHealth(topic :Topic | undefined) :[IndicatorColor, string]{
    if(!topic) {
        return [IndicatorColor.fault, "Topic unkown"];
    }
    if(!isRecent(topic.lastseen, 5)) {
        return [IndicatorColor.fault, "Topic information too old"];
    }
    return getStatisticsHealth(topic.statistics)
}

export function getTopicHealthPubs(topic :Topic | undefined, pubs :TopicPublication[]) :[IndicatorColor, string] {
    let topicHealth = getTopicHealth(topic);
    if(topicHealth[0] !== IndicatorColor.idle) {
        // getTopicHealth calucaltes health based on statistics and on how recent the information is.
        // If that gives conclusive a answer (danger, fault, active) then that takes priority over
        // health calculations based on publications or subscriptions.
        return topicHealth;
    }

    pubs = pubs.filter(p => isRecent(p.lastseen));

    if(pubs.length === 0) {
        return [IndicatorColor.danger, "Topic has no recently known publishers"];
    }

    return [IndicatorColor.idle, `Topic has ${pubs.length} attached publishers`]
}

export interface Node {
    name :string
    lastseen :number
}

export interface TopicPubSub {
    nodename :string
    topicname :string
    lastseen :number
}

export function eqTopicPubSub(a :TopicPubSub, b :TopicPubSub) {
    return a.lastseen === b.lastseen && a.nodename === b.nodename && a.topicname === b.topicname;
}

export function eqTopicPubSubs(a :TopicPubSub[], b :TopicPubSub[]) {
    return a.findIndex((aa, i) => !eqTopicPubSub(aa, b[i])) === -1;
}

export interface TopicPublication extends TopicPubSub {}

export interface TopicSubscription extends TopicPubSub {}

export interface TopicStatistic {
    traffic :number
    lastseen :number
}

export function eqStatistics(a :TopicStatistic | null, b :TopicStatistic | null) {
    if(a === b) {
        return true;
    }

    if(a === null || b === null) {
        return false;
    }

    return a.lastseen === b.lastseen && a.traffic === b.traffic;
}

export function getStatisticsHealth(ts :TopicStatistic | null) :[IndicatorColor, string] {
    if(ts == null) {
        return [IndicatorColor.idle, "No statistics available, assuming idle"]
    }
    if(!isRecent(ts.lastseen, 5)) {
        return [IndicatorColor.idle, "No recent statistics available, assuming idle"]
    }
    if(ts.traffic === 0) {
        return [IndicatorColor.danger, "Recent statistics show 0 msg/s"]
    }
    return [IndicatorColor.active, ts.traffic + " msg/s"];
}

export interface Recording {
    is_recording :boolean
    config_topics :string[]
    config_filename :string
    recording_file :string
    recording_duration :number
    recording_filesize :number
    recording_topics :string[]
    lastrefresh_recording :number
    lastrefresh_config :number
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