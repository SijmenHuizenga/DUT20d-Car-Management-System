import React from 'react';
import TimebasedIndicator from "../util/TimebasedIndicator";

class NodesBlock extends React.Component {
    render() {
        return <div className="block y-50">
            {Object.keys(this.props.nodes).map((nodename) =>
                <NodeIndicator
                    key={nodename}
                    node={nodename}
                    {...this.props.nodes[nodename]} />
            )}
        </div>
    }
}

class NodeIndicator extends React.Component {
    render() {
        let {node, lastseen} = this.props;
        return (
            <div>
                <span className="text-small">
                    <TimebasedIndicator
                        hover="Last seen {timesincelastseen} seconds ago"
                        timestamp={lastseen} success={true} />
                    <span className="pl-1">{node}</span>
                </span>
            </div>
        );
    }
}

export default NodesBlock;