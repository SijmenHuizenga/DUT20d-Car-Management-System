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
    rebootbtnDisabled: boolean
}

class ComputeboxBlock extends React.Component<Props, State> {

    constructor(props :Props) {
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
        return <Indicator color={connected ? IndicatorColor.active : IndicatorColor.danger}
                                 dataTimestamp={lastping}
                                 tooltip={`SSH ${connected ? "connected" : "disconnected"}.`}/>
    }

    renderPingIndicator() {
        let {timestamp, success} = this.props.ping;
        return <Indicator color={success ? IndicatorColor.active : IndicatorColor.danger}
                          dataTimestamp={timestamp}
                          tooltip={`Last ping ${success ? "succeeded" : "failed"}`}/>
    }

    renderRosnodeIndicator() {
        let up = this.props.rosnode_up;
        return <Tooltip tooltip={`Ros connection ${up ? "up" : "down"}.`}>
            <Indicator color={up ? IndicatorColor.active : IndicatorColor.danger} />
        </Tooltip>
    }

    rebootLuke() {
        this.setState({
            rebootbtnDisabled: true
        });
        Requestor.execute("/rebootluke", "POST")
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
}

export default ComputeboxBlock;