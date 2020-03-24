import React from 'react';
import Requestor from "../util/Requestor";
import Tooltip from "../util/Tooltip";
import {Ping, SSH} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";
import {toast} from "react-toastify";

interface Props {
    ssh : SSH
    ping :Ping
    rosnode_up :boolean
}

interface State {
    rebootbtnDisabled: boolean,
    clearPerceptionButtonDisabled: boolean,
}

class ComputeboxBlock extends React.Component<Props, State> {

    constructor(props :Props) {
        super(props);
        this.state = {
            rebootbtnDisabled: false,
            clearPerceptionButtonDisabled: false,
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
                {(this.props.ssh || {uptime: "SSH not started"}).uptime}</span>
            <div className="float-right">
                <button type="button"
                        className="btn btn-sm btn-outline-primary py-0 mr-1"
                        disabled={this.state.clearPerceptionButtonDisabled}
                        onClick={this.clearPerception.bind(this)}>Clear perception cache
                </button>
                <button type="button"
                        className="btn btn-sm btn-outline-danger py-0"
                        disabled={this.state.rebootbtnDisabled}
                        onClick={this.rebootLuke.bind(this)}>Reboot Luke
                </button>
            </div>
        </div>
    }

    renderSshIndicator() {
        if(this.props.ssh === undefined) {
            return null
        }
        let {connected, lastping} = this.props.ssh;
        return <Indicator color={connected ? IndicatorColor.active : IndicatorColor.danger}
                                 dataTimestamp={lastping}
                                 tooltip={this.renderSshTooltip}/>
    }

    renderSshTooltip = () => {
        return `SSH ${this.props.ssh.connected ? "connected" : "disconnected"}.`;
    };

    renderPingIndicator() {
        if(this.props.ping === undefined) {
            return null
        }
        let {timestamp, success} = this.props.ping;
        return <Indicator color={success ? IndicatorColor.active : IndicatorColor.danger}
                          dataTimestamp={timestamp}
                          tooltip={this.renderPingTooltip}/>
    }

    renderPingTooltip = () => {
        return `Last ping from cms groundstation to luke ${this.props.ping.success ? "succeeded" : "failed"}`;
    };

    renderRosnodeIndicator() {
        if(this.props.rosnode_up === undefined) {
            return null
        }
        let up = this.props.rosnode_up;
        return <Tooltip tooltip={() => `Ros connection ${up ? "up" : "down"}.`}>
            <Indicator color={up ? IndicatorColor.active : IndicatorColor.danger} />
        </Tooltip>
    }

    rebootLuke() {
        this.setState({
            rebootbtnDisabled: true
        });
        Requestor.runcommand("sudo reboot")
            .then(() =>
                toast("Reboot started")
            )
            .catch((error) =>
                toast("Reboot fialed: "+error, {type: 'error'})
            ).then(() =>
            this.setState({
                rebootbtnDisabled: false
            }));
    }

    clearPerception() {
        this.setState({
            clearPerceptionButtonDisabled: true
        });
        Requestor.runcommand("rosservice call /perception_server/reset")
            .then(() =>
                toast("Perception cleared")
            )
            .catch((error) =>
                toast("Clearing perception failed: "+error, {type: 'error'})
            ).then(() =>
            this.setState({
                clearPerceptionButtonDisabled: false
            }));
    }
}

export default ComputeboxBlock;