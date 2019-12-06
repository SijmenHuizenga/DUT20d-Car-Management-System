import React from 'react';
import TimebasedIndicator from "../util/TimebasedIndicator";

class TopicsBlock extends React.Component {
    render() {
        return <div className="block y-50">
            {Object.keys(this.props.topics).map((topicname) =>
                <TopicIndicator
                    key={topicname}
                    topic={topicname}
                    {...this.props.topics[topicname]} />)}
        </div>
    }
}

class TopicIndicator extends React.Component {
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