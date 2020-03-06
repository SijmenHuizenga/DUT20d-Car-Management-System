import React from "react";
import Tooltip, {TooltipCreator} from "./Tooltip";
import {OUTDATED_AFTER_SECONDS} from "../statetypes";
import {devmode} from "../Dashboard";

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
    tooltip: TooltipCreator | null
}

export class Indicator extends React.Component<Props, {outdated :boolean}> {
    public static defaultProps = {
        dataTimestamp: null,
        tooltip: null
    };
    private timer: NodeJS.Timeout | undefined;

    constructor(props :Props) {
        super(props);
        this.state = {
            outdated: props.dataTimestamp == null ? false : !isRecent(props.dataTimestamp)
        };
    }

    componentDidMount() {
        this.timer = setInterval(() => {
            if (this.props.dataTimestamp === null) {
                return null;
            }
            this.setState(prevState => {
                const newOutdated = !isRecent(this.props.dataTimestamp as number);
                if (prevState.outdated !== newOutdated) {
                    return {outdated: newOutdated}
                }
            })
        }, 1000);
    }

    componentWillUnmount() {
        clearInterval(this.timer as NodeJS.Timeout);
    }

    static getDerivedStateFromProps = function (props :Props, state :{outdated :boolean}) {
        if(props.dataTimestamp === null) {
            return null;
        }
        return {
            outdated: !isRecent(props.dataTimestamp)
        }
    };

    shouldComponentUpdate(nextProps: Props, nextState: { outdated: boolean }) {
        return nextProps.color !== this.props.color || nextState.outdated !== this.state.outdated || nextProps.tooltip !== this.props.tooltip;
    }

    render() {
        const {tooltip} = this.props;

        if(this.state.outdated) {
            return <Tooltip tooltip={() =>
                        `The information last updated more than ${OUTDATED_AFTER_SECONDS} seconds ago. \n` +
                        `You cannot not trust old data so I turn orange.`}>
                    {this.renderCircle(IndicatorColor.fault)}
                </Tooltip>
        }

        if(tooltip == null) {
            return this.renderCircle()
        }

        return <Tooltip tooltip={tooltip}>
            {this.renderCircle()}
        </Tooltip>
    }

    renderCircle(color = this.props.color) {
        return <span className={`indicator circle mr-1 ${color}`}/>
    }

}

export function isRecent(timestamp :number, secondsRecent = OUTDATED_AFTER_SECONDS) {
    if(devmode) {
        return true;
    }
    return new Date().getTime() / 1000 - timestamp < secondsRecent
}
