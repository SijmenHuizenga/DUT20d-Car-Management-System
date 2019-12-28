import React from 'react';
import {Topic} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";

interface Props {
    topics :{[key: string]: Topic}
}

class TopicsBlock extends React.Component<Props, {}> {
    render() {
        return <div className="block y-50">
            {Object.keys(this.props.topics).map((topicname) =>
                <TopicIndicator
                    key={topicname}
                    topic={topicname}
                    lastseen={this.props.topics[topicname].lastseen} />)}
        </div>
    }
}

interface TopicProps extends Topic {
    topic :string
}

class TopicIndicator extends React.Component<TopicProps, {}> {
    render() {
        let {topic, lastseen} = this.props;
        return <div>
            <Indicator
                color={IndicatorColor.fault}
                tooltip="Work in progress"
                dataTimestamp={lastseen} />
            <span className="pl-1 text-small">{topic}</span>
        </div>
    }
}

export default TopicsBlock;