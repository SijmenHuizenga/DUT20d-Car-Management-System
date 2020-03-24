import React from 'react';
import {
    eqStatistics,
    eqTopicPubSubs,
    getStatisticsHealth,
    Topic,
    TopicPublication,
    TopicPubSub,
    TopicSubscription
} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";

interface Props {
    topics :Topic[]
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
}

class TopicsBlock extends React.Component<Props, {}> {
    render() {
        if(this.props.topics === undefined) {
            return null;
        }

        const sortedTopics = [...this.props.topics].sort((a :Topic, b :Topic) => {
            if((a.statistics === null) === (b.statistics === null)) {
                return a.name.localeCompare(b.name);
            }
            if(a.statistics === null) {
                return 1
            }
            return -1
        });

        return <div className="block y-50">
            <h2>Topics</h2>
            {sortedTopics.map((topic) =>
                <TopicIndicator key={topic.name} {...topic}
                                publications={this.props.subscriptions.filter(sub => sub.topicname === topic.name)}
                                subscriptions={this.props.publications.filter(pub => pub.topicname === topic.name)} />
            )}
        </div>
    }
}

interface TopicInfo extends Topic {
    subscriptions :TopicSubscription[]
    publications :TopicPublication[]
}

class TopicIndicator extends React.Component<TopicInfo, {}> {
    render() {
        let {name, lastseen, statistics} = this.props;

        let statsHealth = getStatisticsHealth(statistics);
        return <div>
            <Indicator
                color={statsHealth ? statsHealth[0] : IndicatorColor.idle}
                tooltip={this.renderTooltip}
                dataTimestamp={lastseen} />
            <span className="pl-1 text-small">{name}</span>
        </div>
    }

    shouldComponentUpdate(nextProps :TopicInfo): boolean {
        return nextProps.name !== this.props.name
            || nextProps.lastseen !== this.props.lastseen
            || !eqStatistics(nextProps.statistics, this.props.statistics)
            || !eqTopicPubSubs(nextProps.subscriptions, this.props.subscriptions)
            || !eqTopicPubSubs(nextProps.publications, this.props.publications);
    }

    renderTooltip = () => {
        let {subscriptions, publications, statistics} = this.props;

        return <div className={"text-small"}>
            {statistics == null ? "No statistics available, asuming idle" : statistics.traffic + " msg/s"}<br/>
            <br/>
            Publishing nodes:
            <ul>
                {subscriptions.length > 0 ? this.renderNodes(subscriptions) : <div>none</div>}
            </ul>
            Subscribing nodes:
            <ul>
                {publications.length > 0 ? this.renderNodes(publications) : <div>none</div>}
            </ul>
        </div>;
    };

    renderNodes(pubsubs :TopicPubSub[]) {
        return pubsubs.map(pb => TopicIndicator.renderNodeIndicator(pb.nodename, pb.lastseen))
    }

    static renderNodeIndicator(nodename :string, pubsubLastseen :number) {
        return <div key={nodename}>
            <Indicator color={IndicatorColor.active} dataTimestamp={pubsubLastseen} />
            <span className="pl-1 text-small">{nodename}</span>
        </div>
    }
}

export default TopicsBlock;