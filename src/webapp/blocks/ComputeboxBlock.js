import React from 'react';
import ReactTooltip from 'react-tooltip'
import Requestor from "../util/Requestor";

class ComputeboxBlock extends React.Component {
    render() {
        let up = this.props.rosnode_up;
        let {timestamp, success, uptime} = this.props.ping;
        let sshConnected = this.props.ssh.connected;

        return <div className="block clearfix">
            <span className="text-small ">
                <span className={"indicator circle " + (up ? "success" : "danger")}
                      data-tip={`Groundstation ros node is ${up ? "up" : "down"} <br /> CMS is connected to ros master.`}/>&nbsp;
                <span className={"indicator circle " + (sshConnected ? "success" : "danger")}
                      data-tip={`Groundstation is ${sshConnected ? "" : "not"} connected to luke via ssh`}/>&nbsp;
                <span className={"indicator circle " + this.getPingColor(timestamp, success)}
                      data-tip data-for="pingtooltip"/>&nbsp;
                {uptime}</span>
            <div className="float-right">
                <button type="button"
                        className="btn btn-sm btn-outline-danger py-0"
                        onClick={this.rebootLuke.bind(this)}>Reboot Luke</button>
            </div>
            <ReactTooltip place="bottom" delayShow={300} id='pingtooltip' getContent={[() => {
                return this.getPingTitle(timestamp, success)
            }, 100]} />
        </div>
    }

    getPingColor(timestamp, success) {
        if(!this.lastPingWasRecent(timestamp)) {
            return "warning";
        }
        return success ? "success" : "danger"
    }

    getPingTitle(timestamp, success) {
        let now = (new Date()).getTime() / 1000;
        if(!this.lastPingWasRecent(timestamp)) {
            return "Last ping to computebox was too long ago ("+this.nicenumber(now - timestamp)+" seconds)"
        }

        return `Last ping to computebox (${this.nicenumber(now - timestamp)} seconds ago) was ${success ? "successfull" : "fail"}. `
    }

    lastPingWasRecent(timestamp) {
        return new Date().getTime() / 1000 - timestamp < 2
    }

    nicenumber(n) {
        n = Math.round(n * 10) / 10;
        if(n % 1 === 0)
            return n + '.0';
        return n + '';
    }

    componentDidMount() {
        //force re-render every 0.5 second to make sure indicators are updated
        this.interval = setInterval(() => this.setState({ time: Date.now() }), 500);
    }
    componentWillUnmount() {
        clearInterval(this.interval);
    }

    rebootLuke() {
        Requestor.execute("/rebootluke")
    }
}

export default ComputeboxBlock;