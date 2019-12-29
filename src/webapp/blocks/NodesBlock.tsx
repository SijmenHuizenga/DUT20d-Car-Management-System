import React from 'react';
import {Node, TopicPublication, TopicPubSub, TopicStatistic, TopicSubscription} from "../statetypes"
import {Indicator, IndicatorColor} from "../util/Indicator";
import {isStatisticHealthy, isNodeActive} from "../util/PubSubSmartness";

interface Props {
    nodes :Node[]
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
    topicstatistics :TopicStatistic[]
}

class NodesBlock extends React.Component<Props> {
    render() {
        return <div className="block y-50">
            {this.props.nodes.map((node) =>
                <NodeIndicator key={node.name} {...node}
                               subscriptions={this.props.subscriptions.filter(sub => sub.nodename === node.name)}
                               publications={this.props.publications.filter(pub => pub.nodename === node.name)}
                               topicstatistics={this.props.topicstatistics.filter(stat => node.name === stat.node_pub || node.name === stat.node_sub)}
                />
            )}
        </div>
    }
}

interface NodeInfo extends Node {
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
    topicstatistics :TopicStatistic[]
}

class NodeIndicator extends React.Component<NodeInfo> {
    render() {
        let {name, lastseen, subscriptions, publications, topicstatistics} = this.props;
        let [isActive, status] = isNodeActive(this.props, subscriptions, publications, topicstatistics);
        return (
            <div>
                <span className="text-small">
                    <Indicator
                        color={isActive ? IndicatorColor.active : IndicatorColor.idle}
                        tooltip={this.renderTooltip(status)}
                        dataTimestamp={lastseen} />
                    <span className="pl-1">{name.substr(1)}</span>
                </span>
            </div>
        );
    }

    renderTooltip(status :string) {
        let {subscriptions, publications, name} = this.props;
        subscriptions = subscriptions.filter(sub => sub.nodename === name);
        publications = publications.filter(pub => pub.nodename === name);
        return <div>
            {status}<br/>
            Subscribed to
            <ul>
                {subscriptions.length > 0 ? this.renderTopicIndicators(subscriptions, 'sub') : <div>none</div>}
            </ul>
            Publishing to
            <ul>
                {publications.length > 0 ? this.renderTopicIndicators(publications, 'pub') : <div>none</div>}
            </ul>
        </div>;
    }

    renderTopicIndicators(subscriptions :TopicPubSub[], pubsub :'pub' | 'sub') {
        return subscriptions.map(sub => this.renderTopicIndicator(sub.topicname, sub.lastseen, pubsub))
    }

    renderTopicIndicator(topicname :string, topicLastseen :number, pubsub :'pub' | 'sub') {
        let [healthy, status] = this.getPubSubTopicHealth(topicname, pubsub);
        return <div key={pubsub+topicname}>
            <Indicator color={healthy} dataTimestamp={topicLastseen} />
            <span className="pl-1 text-small">{topicname} ({status})</span>
        </div>
    }

    // Get the health of a topic that is either published by or subscribed to this node.
    getPubSubTopicHealth(topicname :string, pubsub :'pub' | 'sub') :[IndicatorColor, string]{
        let stats = this.props.topicstatistics.filter(
            (stat) => stat[pubsub == 'pub' ? 'node_pub' : 'node_sub'] === this.props.name && stat.topic === topicname);
        return stats.length === 0 ? [IndicatorColor.idle, "No statistics available"] : isStatisticHealthy(stats, pubsub);
    }
}

export default NodesBlock;