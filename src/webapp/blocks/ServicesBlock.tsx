import React from "react";
import Tooltip from "../util/Tooltip";
import {nicenumber} from "../util/Timing";
import {SystemdService} from "../statetypes";

class ServicesBlock extends React.Component<{systemdservices: {[key: string]: SystemdService}}, {}> {
    render() {
        let services = this.props.systemdservices;
        return <div className="block">
            {Object.keys(services).map((servicename) =>
                <ServiceIndicator {...services[servicename]} name={servicename}/>
            )}
        </div>
    }
}

interface IndicatorProps extends SystemdService{
    name :string
}

class ServiceIndicator extends React.Component<IndicatorProps, {}> {
    render() {
        let {name, statustext} = this.props;
        return (
            <span className="text-small text-nowrap pr-2">
                {this.renderIndicator()}
                <Tooltip tooltip={statustext.trim()}>{name}</Tooltip>
                {this.props.status !== "error"
                    ? <span className="grayyed">({this.renderEnabledDisabled()})</span>
                    : null
                }
            </span>
        );
    }

    renderIndicator() {
        let secondsSinceLastUpdate = new Date().getTime() / 1000 - this.props.lastupdate;
        let wasRecent = secondsSinceLastUpdate < 10;

        return <Tooltip tooltip={wasRecent ? `Service is ${this.props.status}. \n Last update ${nicenumber(secondsSinceLastUpdate)} seconds ago.`  : "Last check was too long ago."}>
            <span className={`indicator circle mr-1 ${wasRecent ? this.getIndicatorColor() : "warning"}`}/>
        </Tooltip>
    }

    getIndicatorColor() {
        switch (this.props.status) {
            case "running":
                return "success";
            case "stopped":
                return "warning";
            default:
                return "danger"
        }
    }

    renderEnabledDisabled() {
        if (this.props.enabled) {
            return <Tooltip tooltip={"This service auto-starts on boot"}>enabled</Tooltip>
        }
        return <Tooltip tooltip={"This service does not start on boot"}>disabled</Tooltip>
    }
}

export default ServicesBlock;

