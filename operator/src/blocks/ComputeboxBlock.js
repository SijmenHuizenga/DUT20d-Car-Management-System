import React from 'react';
import ReactTooltip from 'react-tooltip'

class ComputeboxBlock extends React.Component {
    render() {
        let {rosnode: {up}, pinger: {computebox: {last_success, last_run}}} = this.props.groundStationState;

        return <div className="block clearfix">
            <span className="text-small ">
                <span className={"indicator circle " + (up ? "success" : "danger")}
                      data-tip={"Groundstation ros node is " + (up ? "up" : "down")}/>&nbsp;
                <span className={"indicator circle " + (this.getPingColor(last_success))}
                      data-tip data-for="pingtooltip"/>&nbsp;
                Uptime: 25 minutes</span>
            <div className="float-right">
                <button type="button" className="btn btn-sm btn-outline-danger py-0">Reboot Luke</button>
            </div>
            <ReactTooltip place="bottom" />
            <ReactTooltip place="bottom" id='pingtooltip' getContent={[() => {
                return this.getPingTitle(last_success, last_run)
            }, 100]} />
        </div>
    }

    getPingColor(last_success) {
        if(new Date().getTime() / 1000 - last_success < 2) {
            return "success";
        } else {
            return "danger"
        }
    }

    getPingTitle(last_success, last_run) {
        let now = Math.round((new Date()).getTime() / 100);
        return "Last ping to computebox was " + Math.round(now - last_run * 10) / 10 + " seconds ago. " +
            "Last successful ping was " + Math.round(now - last_success * 10) / 10 + " seconds ago."
    }

    componentDidMount() {
        //force re-render every 0.5 second to make sure indicators are updated
        this.interval = setInterval(() => this.setState({ time: Date.now() }), 500);
    }
    componentWillUnmount() {
        clearInterval(this.interval);
    }
}

export default ComputeboxBlock;