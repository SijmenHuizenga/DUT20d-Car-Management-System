import React from "react";
import Tooltip from "./Tooltip";
import {OUTDATED_AFTER_SECONDS} from "../statetypes";

export enum IndicatorColor {
    // Danger = Something is wrong and requires immediate action from a user
    //   A process, service or node crashed.
    //   A topic that is being recorded is not sending any data.
    danger = "danger",

    // Orange = Fault: CMS is unsure about the status.
    //   The last received update about something took place a long time ago
    //   Cms received unknown responses from ros
    //   Cms has a bug.
    fault = "fault",

    // Green = Active = Something is actively in use.
    //   A process, service or node is running.
    //   A topic is actively processing messages.
    //   A network device is reachable.
    active = "active",

    // Gray = IDLE = Idle: Something exists but not running.
    //   This color is only used for things that can exist without running.
    //   A systemd service is stopped.
    //   A topic exists but is not receiving messages.
    idle = "idle",
}

interface Props {
    color :IndicatorColor

    // The timestamp the information visualised by this indicator was last updated.
    // If this value is null we assume the data is valid forever.
    dataTimestamp :number | null

    // The tooltip. If null no tooltip is shown.
    // When the information is outdated according to dataTimestamp a tooltip will always be shown.
    tooltip :string | null | JSX.Element
}

export class Indicator extends React.PureComponent<Props> {
    public static defaultProps = {
        dataTimestamp: null,
        tooltip: null
    };

    render() {
        const {dataTimestamp, tooltip} = this.props;
        const isDataRecent = dataTimestamp == null ? true : isRecent(dataTimestamp);

        if(!isDataRecent) {
            return <Tooltip tooltip={
                        `The information last updated more than ${OUTDATED_AFTER_SECONDS} seconds ago. \n` +
                        `You cannot not trust old data so I turn orange.`}>
                    {this.renderCircle(IndicatorColor.fault)}
                </Tooltip>
        }

        if(tooltip == null) {
            return this.renderCircle()
        }

        return <Tooltip tooltip={this.props.tooltip!}>
                {this.renderCircle()}
            </Tooltip>
    }

    renderCircle(color = this.props.color) {
        return <span className={`indicator circle mr-1 ${color}`}/>
    }

}

export function isRecent(timestamp :number, secondsRecent = OUTDATED_AFTER_SECONDS) {
    return new Date().getTime() / 1000 - timestamp < secondsRecent
}