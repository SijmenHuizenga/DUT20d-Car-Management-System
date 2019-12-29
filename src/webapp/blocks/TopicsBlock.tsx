import React from 'react';
import {Topic} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";

interface Props {
    topics :Topic[]
}

class TopicsBlock extends React.Component<Props, {}> {
    render() {
        return <div className="block y-50">
            {this.props.topics.map((topic) =>
                <TopicIndicator key={topic.name} {...topic} />)}
        </div>
    }
}

class TopicIndicator extends React.Component<Topic, {}> {
    render() {
        let {name, lastseen} = this.props;
        return <div>
            <Indicator
                color={IndicatorColor.fault}
                tooltip="Work in progress"
                dataTimestamp={lastseen} />
            <span className="pl-1 text-small">{name}</span>
        </div>
    }
}

export default TopicsBlock;