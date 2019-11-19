import React from 'react';

class ComputeboxBlock extends React.Component {
    render() {
        return <div className="block clearfix">
            <span className="text-small "><span className="indicator circle success" /> Uptime: 25 minutes</span>
            <div className="float-right">
                <button type="button" className="btn btn-sm btn-outline-danger py-0">Reboot Luke</button>
            </div>
        </div>
    }
}

export default ComputeboxBlock;