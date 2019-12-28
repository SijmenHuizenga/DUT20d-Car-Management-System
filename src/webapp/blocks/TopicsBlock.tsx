import React from 'react';
import TimebasedIndicator from "../util/TimebasedIndicator";

interface Topic {
    lastseen :number
}

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

class TopicIndicator extends React.Component<{topic :string, lastseen :number}, {}> {
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