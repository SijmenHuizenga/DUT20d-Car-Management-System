import React from "react";
import Tooltip from "../util/Tooltip";
import {SystemdService, SystemdServiceRunning, SystemdServiceEnabled} from "../statetypes";
import {ContextMenu, ContextMenuTrigger, MenuItem} from "react-contextmenu";
import {Indicator, IndicatorColor} from "../util/Indicator";
import Requestor from "../util/Requestor";
import {toast} from "react-toastify";

class ServicesBlock extends React.Component<{ systemdservices: SystemdService[] }, {}> {
    render() {
        return <div className="block">
            {this.props.systemdservices.map((service) =>
                <ServiceIndicator key={service.name} {...service}/>
            )}
        </div>
    }
}

class ServiceIndicator extends React.Component<SystemdService> {

    render() {
        let {name, statustext} = this.props;
        return (<ContextMenuTrigger id={`service${this.props.name}startstoptrigger`}>
            <span className="text-small text-nowrap pr-2">
                <Indicator color={this.getIndicatorColor()}
                           tooltip={this.getIndicatorTooltipText()}
                           dataTimestamp={this.props.lastupdate}/>
                <Tooltip tooltip={statustext.trim()}>{name.substr(0, name.length-8)}</Tooltip>
                &nbsp;
                <span className="text-grayyed">({this.renderEnabledDisabled()})</span>
                {this.renderContextmenu()}
            </span>
        </ContextMenuTrigger>);
    }

    getIndicatorColor() {
        switch (this.props.running) {
            case SystemdServiceRunning.running:
                return IndicatorColor.active;
            case SystemdServiceRunning.stopped:
                return IndicatorColor.idle;
            case SystemdServiceRunning.error:
                return IndicatorColor.danger;
            default:
                return IndicatorColor.fault
        }
    }

    getIndicatorTooltipText() {
        return `Service is ${this.props.running}.`
    }

    renderEnabledDisabled() {
        switch (this.props.enabled) {
            case SystemdServiceEnabled.enabled:
                return <Tooltip tooltip={"This service auto-starts on boot"}>enabled</Tooltip>;
            case SystemdServiceEnabled.disabled:
                return <Tooltip tooltip={"This service does not start on boot"}>disabled</Tooltip>;
            case SystemdServiceEnabled.error:
                return <Tooltip tooltip={"Could not retrieve enabled/disabled status"}>unkown</Tooltip>;
        }
    }

    renderContextmenu() {
        return <ContextMenu id={`service${this.props.name}startstoptrigger`}>
            {this.renderActivateButtons()}
            {this.renderMaskButtons()}
        </ContextMenu>
    }

    renderActivateButtons() {
        switch (this.props.running) {
            case SystemdServiceRunning.running:
                return <React.Fragment>
                    <MenuItem onClick={() => this.handleClick(`sudo systemctl restart ${this.props.name}`, `Restarting ${this.props.name}`)}>
                        Restart
                    </MenuItem>
                    <MenuItem onClick={() => this.handleClick(`sudo systemctl stop ${this.props.name}`, `Stopping ${this.props.name}`)}>
                        Stop
                    </MenuItem>
                </React.Fragment>;
            case SystemdServiceRunning.stopped:
                return <MenuItem onClick={() => this.handleClick(`sudo systemctl start ${this.props.name}`, `Starting ${this.props.name}`)}>
                    Start
                </MenuItem>;
            default:
                return <MenuItem>"Service status unkonwn, no actions available"</MenuItem>
        }
    }

    private renderMaskButtons() {
        if(this.props.running === SystemdServiceRunning.error) {
            return null
        }
        if(this.props.enabled === SystemdServiceEnabled.enabled) {
            return <MenuItem onClick={() => this.handleClick(`sudo systemctl disable ${this.props.name}`, `Disabling ${this.props.name}`)}>
                    Disable (Make service no longer auto-start on boot)
                </MenuItem>
        } else if (this.props.enabled === SystemdServiceEnabled.disabled){
            return <MenuItem onClick={() => this.handleClick(`sudo systemctl enable ${this.props.name}`, `Enabling ${this.props.name}`)}>
                Enable (Make service will auto-start on boot)
            </MenuItem>
        } else {
            return null
        }
    }

    handleClick(command :string, title :string) {
        const toastid = toast(`${title}...`, { autoClose: false });
        Requestor.execute("/runcommand", "POST", {command})
            .then((body :any) => body.json())
            .then((body :any) => {
                toast.update(toastid, {
                    render: <span title={body.output}>{title} {body.statuscode === 0 ? 'ok' : 'failed'}</span>,
                    type: body.statuscode === 0 ? toast.TYPE.SUCCESS : toast.TYPE.ERROR,
                    autoClose: body.statuscode === 0 ? 5000 : 0
                })}
            )
            .catch((error) => toast.update(toastid, {
                render: <span title={error}>{title} error</span>,
                type: toast.TYPE.WARNING,
            }))
    }
}

export default ServicesBlock;

