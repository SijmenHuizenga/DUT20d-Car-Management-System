import React from "react";

class ServicesBlock extends React.Component {
    render() {
        return <div className="block">
            <span className="text-small  text-nowrap pr-2"><span
                  className="indicator circle success" /> roscore (enabled)</span>
            <span className="text-small  text-nowrap pr-2"><span
                className="indicator circle success" /> inspection (enabled)</span>
            <span className="text-small  text-nowrap pr-2"><span
                className="indicator circle success" /> maveros (enabled)</span>
            <span className="text-small  text-nowrap pr-2"><span
                className="indicator circle success" /> timesync (enabled)</span>
            <span className="text-small  text-nowrap pr-2"><span
                className="indicator circle success" /> p2p (disabled)</span>
            <span className="text-small  text-nowrap pr-2"><span
                className="indicator circle success" /> one4all (disabled)</span>
            <span className="text-small  text-nowrap pr-2"><span
                className="indicator circle success" /> recorder (disabled)</span>
        </div>
    }
}

export default ServicesBlock;

