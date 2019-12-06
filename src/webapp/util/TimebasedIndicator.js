import React from 'react';
import {formatUnicorn, indicatorColorBasedOnTime, nicenumber} from "./Timing";
import Tooltip from "./Tooltip";

class TimebasedIndicator extends React.Component {

    render() {
        let {timestamp, success} = this.props;
        return (
            <Tooltip tooltip={this.getTitle()}>
                <div className="d-inline">
                    <span className={"indicator circle " + indicatorColorBasedOnTime(timestamp, success)}/>
                </div>
            </Tooltip>
        )
    }

    getTitle() {
        let now = (new Date()).getTime() / 1000;
        return formatUnicorn(this.props.hover, {...this.props, timesincelastseen: nicenumber(now - this.props.timestamp)})
    }

    componentDidMount() {
        //force re-render every 0.5 second to make sure indicator colors are updated
        this.interval = setInterval(() => this.setState({ time: Date.now() }), 100);
    }

    componentWillUnmount() {
        clearInterval(this.interval);
    }
}

export default TimebasedIndicator;