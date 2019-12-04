import React from 'react';

class TopicsBlock extends React.Component {
    render() {
        return <div className="block y-50">{Object.keys(this.props.topics).map((topicname) =>
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
            <span className="indicator circle success"/>
            <span className="pl-1">{topic}</span>
        </div>
    }
}

export default TopicsBlock;