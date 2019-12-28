import React from "react";
import Tooltip from "../util/Tooltip";
import {SystemdService, SystemdServiceRunning, SystemdServiceEnabled, OUTDATED_AFTER_SECONDS} from "../statetypes";
import {ContextMenu, ContextMenuTrigger, MenuItem} from "react-contextmenu";
import {Indicator, IndicatorColor} from "../util/Indicator";

class ServicesBlock extends React.Component<{ systemdservices: SystemdService[] }, {}> {
    render() {
        return <div className="block">
            {this.props.systemdservices.map((service) =>
                <ServiceIndicator {...service}/>
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
                <Tooltip tooltip={statustext.trim()}>{name}</Tooltip>
                <span className="grayyed">({this.renderEnabledDisabled()})</span>
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
                    <MenuItem onClick={() => this.handleClick(`systemctl restart ${this.props.name}`)}>
                        Restart
                    </MenuItem>
                    <MenuItem onClick={() => this.handleClick(`systemctl stop ${this.props.name}`)}>
                        Stop
                    </MenuItem>
                </React.Fragment>;
            case SystemdServiceRunning.stopped:
                return <MenuItem onClick={() => this.handleClick(`systemctl start ${this.props.name}`)}>
                    Start
                </MenuItem>;
            default:
                return null
        }
    }

    private renderMaskButtons() {
        if(this.props.running === SystemdServiceRunning.error) {
            return null
        }
        if(this.props.enabled) {
            return <MenuItem onClick={() => this.handleClick(`systemctl disable ${this.props.name}`)}>
                    Disable (Make service no longer auto-start on boot)
                </MenuItem>
        } else {
            return <MenuItem onClick={() => this.handleClick(`systemctl enable ${this.props.name}`)}>
                Enable (Make service will auto-start on boot)
            </MenuItem>
        }
    }

    handleClick(command :string) {
        console.log("work in progress")
    }
}

export default ServicesBlock;

