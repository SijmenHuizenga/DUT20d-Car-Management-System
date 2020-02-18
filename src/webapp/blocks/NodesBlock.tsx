import React from 'react';
import {
    eqTopicPubSubs,
    getTopicHealth,
    Node,
    Topic,
    TopicPublication,
    TopicPubSub,
    TopicSubscription
} from "../statetypes"
import {Indicator, IndicatorColor} from "../util/Indicator";

interface Props {
    nodes :Node[]
    topics :Topic[]
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
}

class NodesBlock extends React.Component<Props> {
    render() {
        if(this.props.nodes === undefined) {
            return null;
        }

        const sortedNodes = [...this.props.nodes].sort((a :Node, b :Node) => a.name.localeCompare(b.name));

        return <div className="block y-50">
            {sortedNodes.map(this.renderNodeIndicator.bind(this))}
        </div>
    }

    renderNodeIndicator(node :Node) {
        let subs = this.props.subscriptions.filter(sub => sub.nodename === node.name);
        let pubs = this.props.publications.filter(pub => pub.nodename === node.name);

        let relevantTopics = this.props.topics.filter((topic) =>
            subs.find((sub) => sub.topicname === topic.name)
            || pubs.find((pub) => pub.topicname === topic.name));

        return <NodeIndicator key={node.name} {...node}
                              subscriptions={subs}
                              publications={pubs}
                              topics={relevantTopics}
        />
    }
}

interface NodeInfo extends Node {
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
    topics :Topic[]
}

class NodeIndicator extends React.Component<NodeInfo> {
    render() {
        let {name, lastseen} = this.props;
        return (
            <div>
                <span className="text-small">
                    <Indicator
                        color={IndicatorColor.active}
                        tooltip={this.renderTooltip}
                        dataTimestamp={lastseen} />
                    <span className="pl-1">{name.substr(1)}</span>
                </span>
            </div>
        );
    }

    shouldComponentUpdate(nextProps :NodeInfo): boolean {
        return nextProps.name !== this.props.name
            || nextProps.lastseen !== this.props.lastseen
            || !eqTopicPubSubs(nextProps.subscriptions, this.props.subscriptions)
            || !eqTopicPubSubs(nextProps.publications, this.props.publications)
    }

    renderTooltip = () => {
        let {subscriptions, publications, name} = this.props;
        subscriptions = subscriptions.filter(sub => sub.nodename === name);
        publications = publications.filter(pub => pub.nodename === name);
        return <div className={"text-small"}>
            Subscribed to
            <ul>
                {subscriptions.length > 0 ? this.renderTopicIndicators(subscriptions) : <div>none</div>}
            </ul>
            Publishing to
            <ul>
                {publications.length > 0 ? this.renderTopicIndicators(publications) : <div>none</div>}
            </ul>
        </div>;
    };

    renderTopicIndicators(pubsubs :TopicPubSub[]) {
        return pubsubs.map(pb => this.renderTopicIndicator(pb.topicname, pb.lastseen))
    }

    renderTopicIndicator(topicname :string, pubsubLastSeen :number) {
        let [healthy, status] = getTopicHealth(this.props.topics.find((t) => t.name === topicname));
        return <div key={topicname}>
            <Indicator color={healthy} dataTimestamp={pubsubLastSeen} />
            <span className="pl-1 text-small">{topicname} <span className="text-grayyed">({status})</span></span>
        </div>
    }

}

export default NodesBlock;