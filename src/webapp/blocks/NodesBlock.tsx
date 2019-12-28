import React from 'react';
import {Node} from "../statetypes"
import {Indicator, IndicatorColor} from "../util/Indicator";

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

interface IndicatorProps extends Node {
    node :string
}

class NodeIndicator extends React.Component<IndicatorProps, {}> {
    render() {
        let {node, lastseen} = this.props;
        return (
            <div>
                <span className="text-small">
                    <Indicator
                        color={IndicatorColor.fault}
                        tooltip="Work in progress"
                        dataTimestamp={lastseen} />
                    <span className="pl-1">{node.substr(1)}</span>
                </span>
            </div>
        );
    }
}

export default NodesBlock;