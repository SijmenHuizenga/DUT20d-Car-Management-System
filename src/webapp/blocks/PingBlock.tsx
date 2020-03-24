import React from 'react';
import {Ping} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";
import Tooltip from "../util/Tooltip";

interface Props {
    pings : {[key :string] :Ping}
}

class PingBlock extends React.Component<Props, {}> {

    render() {
        return <div className="block">
            <h2>Ping from Luke</h2>
            {this.props.pings == null ? null : this.renderPings()}
        </div>
    }

    renderPings() {
        return Object.values(this.props.pings).map((ping) =>
            <div key={ping.ip}>
                <Tooltip tooltip={() => ping.ip}>
                    <Indicator color={ping.success ? IndicatorColor.active : IndicatorColor.danger} dataTimestamp={ping.timestamp}/>
                    {ping.friendlyname}
                </Tooltip>
            </div>
        )
    }

}

export default PingBlock;