import React from 'react';
import {Node, TopicPublication, TopicSubscription} from "../statetypes"
import {Indicator, IndicatorColor} from "../util/Indicator";

interface Props {
    nodes :Node[]
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
}

class NodesBlock extends React.Component<Props> {
    render() {
        return <div className="block y-50">
            {this.props.nodes.map((node) =>
                <NodeIndicator key={node.name} {...node}
                               subscriptions={this.props.subscriptions.filter(sub => sub.nodename == node.name)}
                               publications={this.props.publications.filter(pub => pub.nodename == node.name)}
                />
            )}
        </div>
    }
}

interface NodeInfo extends Node {
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
}

class NodeIndicator extends React.Component<NodeInfo> {
    render() {
        let {name, lastseen} = this.props;
        return (
            <div>
                <span className="text-small">
                    <Indicator
                        color={IndicatorColor.active}
                        tooltip={this.getStatusDescription()}
                        dataTimestamp={lastseen} />
                    <span className="pl-1">{name.substr(1)}</span>
                </span>
            </div>
        );
    }

    getStatusDescription() {
        let out = "Active: This node was seen recently in the list of publishes and subscribers retreived from ros master.";
        if(this.props.subscriptions.length > 0){
            out += "\nSubscriptions: \n";
            out += this.props.subscriptions.map(sub => `  ${sub.topicname}`).join("\n");
        }
        if(this.props.publications.length > 0) {
            out += "\nPublications: \n";
            out += this.props.publications.map(sub => `  ${sub.topicname}`).join("\n");
        }
        return out;
    }
}

export default NodesBlock;