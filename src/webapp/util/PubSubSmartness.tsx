import {Node, TopicPublication, TopicStatistic, TopicSubscription} from "../statetypes";
import {IndicatorColor, isRecent} from "./Indicator";

export function isNodeActive(node :Node,
                             subscriptions :TopicSubscription[],
                             publications :TopicPublication[],
                             statistics :TopicStatistic[]) : [boolean, string] {
    let filteredSubs = subscriptions.filter(nondefaultTopics).filter(onlyRecent);
    let filteredPubs = publications.filter(nondefaultTopics).filter(onlyRecent);
    if(filteredSubs.length === 0 && filteredPubs.length === 0) {
        return [false, "This node is not subscribed or published to any topics."]
    }

    filteredSubs = filteredSubs.filter(onlyActiveTopics(statistics, 'sub'));
    filteredPubs = filteredPubs.filter(onlyActiveTopics(statistics, 'pub'));
    if(filteredSubs.length === 0 && filteredPubs.length === 0) {
        return [false, "This node is not processing messages (no active topics attached)."]
    }

    return [true, "This node is actively working with topics "]
}

function nondefaultTopics(topic : {topicname :string}) {
    return topic.topicname !== "/rosout";
}

function onlyRecent(topic : {lastseen :number}) {
    return isRecent(topic.lastseen)
}

function onlyActiveTopics(statistics :TopicStatistic[], pubsub :'pub' | 'sub') {
    return (topic : {topicname :string}) => {
        let [healty] = isStatisticHealthy(statistics.filter((s) => s.topic === topic.topicname), pubsub)
        return healty === IndicatorColor.active;
    }
}

export function isStatisticHealthy(stats :TopicStatistic[], pubsub :'pub' | 'sub') :[IndicatorColor, string] {
    //todo: add to messages what timestamp information comes from

    let recentStats = stats.filter((s) => isRecent(s.lastseen, 70));
    if(recentStats.length === 0) {
        return [IndicatorColor.idle, "Statistics outdated, assuming idle"]
    }

    let totalDroppedMessages = recentStats.reduce((counter, b) => counter + b.dropped_msgs, 0);
    if(totalDroppedMessages > 0) {
        return [IndicatorColor.danger, totalDroppedMessages + " dropped messages"]
    }

    let allMsgsSpeed = recentStats.map(statisticsMsgsPerSecond);

    let totalMsgsSpeed = allMsgsSpeed.reduce((a, b) => a + b, 0);

    if(pubsub ==="pub") {
        // statistics always go from source node to target node.
        // so when we filter 'all statistics about node a publishing on topic b'
        // we get n statistics where n is the amount of subscribers to b.
        // To get a correct value of amount of messages sent we must devide by the amount of target nodes.
        totalMsgsSpeed = totalMsgsSpeed / recentStats.length;
    }

    return [IndicatorColor.active, totalMsgsSpeed + " msg/s"];
}

function statisticsMsgsPerSecond(stat :TopicStatistic) {
    return Math.round(stat.delivered_msgs / (stat.window_stop - stat.window_start))
}