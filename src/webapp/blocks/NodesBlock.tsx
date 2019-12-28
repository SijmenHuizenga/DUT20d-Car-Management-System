import React from 'react';
import TimebasedIndicator from "../util/TimebasedIndicator";

interface Node {
    lastseen :number
}

interface Props {
    nodes :{[key: string]: Node}
}

class NodesBlock extends React.Component<Props, {}> {
    render() {
        return <div className="block y-50">
            {Object.keys(this.props.nodes).map((nodename) =>
                <NodeIndicator
                    key={nodename}
                    node={nodename}
                    lastseen={this.props.nodes[nodename].lastseen} />
            )}
        </div>
    }
}

class NodeIndicator extends React.Component<{node :string, lastseen :number}, {}> {
    render() {
        let {node, lastseen} = this.props;
        return (
            <div>
                <span className="text-small">
                    <TimebasedIndicator
                        hover="Last seen {timesincelastseen} seconds ago"
                        timestamp={lastseen} success={true} />
                    <span className="pl-1">{node.substr(1)}</span>
                </span>
            </div>
        );
    }
}

export default NodesBlock;