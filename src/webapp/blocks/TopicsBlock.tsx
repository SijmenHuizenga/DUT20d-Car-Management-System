import React from 'react';
import TimebasedIndicator from "../util/TimebasedIndicator";
import {Topic} from "../statetypes";

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
            <TimebasedIndicator
                hover="Last seen {timesincelastseen} seconds ago"
                timestamp={lastseen} success={true} />
            <span className="pl-1 text-small">{topic}</span>
        </div>
    }
}

export default TopicsBlock;