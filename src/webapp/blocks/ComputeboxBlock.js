import React from 'react';
import Requestor from "../util/Requestor";
import TimebasedIndicator from "../util/TimebasedIndicator";
import Tooltip from "../util/Tooltip";

class ComputeboxBlock extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            rebootbtnDisabled: false
        }
    }

    render() {
        return <div className="block clearfix">
            <span className="text-small ">
                {this.renderPingIndicator()}
                &nbsp;
                {this.renderRosnodeIndicator()}
                &nbsp;
                {this.renderSshIndicator()}
                &nbsp;
                {this.props.ssh.uptime}</span>
            <div className="float-right">
                <button type="button"
                        className="btn btn-sm btn-outline-danger py-0"
                        disabled={this.state.rebootbtnDisabled}
                        onClick={this.rebootLuke.bind(this)}>Reboot Luke
                </button>
            </div>
        </div>
    }

    renderSshIndicator() {
        let {connected, lastping} = this.props.ssh;
        return <TimebasedIndicator timestamp={lastping} success={connected}
                                   hover={`SSH ${connected ? "connected" : "disconnected"}. Last seen: {timesincelastseen}`}/>
    }

    renderPingIndicator() {
        let {timestamp, success} = this.props.ping;
        return <TimebasedIndicator timestamp={timestamp} success={success}
                                   hover={`Ping {timesincelastseen} seconds ago was ${success ? "successfull" : "fail"}`}/>
    }

    renderRosnodeIndicator() {
        let up = this.props.rosnode_up;
        return <Tooltip tooltip={`Ros connection ${up ? "up" : "down"}.`}>
            <span className={"indicator circle " + (up ? "success" : "danger")}/>
        </Tooltip>
    }

    rebootLuke() {
        this.setState({
            rebootbtnDisabled: true
        });
        Requestor.execute("/rebootluke")
            .then(() =>
                alert("Reboot initialized")
            )
            .catch((error) =>
                alert("Reboot failed: \n" + error)
            ).then(() =>
            this.setState({
                rebootbtnDisabled: false
            }));
    }
}

export default ComputeboxBlock;