import React from 'react';

class HealthBlock extends React.Component {

    render() {
        return (
            <div className="block y-50">
                <div>
                    <span className="text-large text-success">5</span>
                    <span className="text-small "> Yelow cones detected</span>
                </div>
                <div>
                    <span className="text-large text-success">7</span>
                    <span className="text-small "> Blue cones detected</span>
                </div>
                <div>
                    <span className="text-large text-success">OK</span>
                    <span className="text-small "> Ping github</span>
                </div>
                <div>
                    <span className="text-large text-warning">54c</span>
                    <span className="text-small "> CPU Temperature</span>
                </div>
                <div>
                    <span className="text-large text-danger">42c</span>
                    <span className="text-small "> Max Disk Temperature</span>
                </div>
                <div>
                    <span className="text-large text-success">223</span>
                    <span className="text-small "> GPS Satilites</span>
                </div>
                <div>
                    <span className="text-large text-success">OK</span>
                    <span className="text-small "> CAN bus</span>
                </div>
                <div>
                    <span className="text-large text-warning">6/7</span>
                    <span className="text-small "> Network ports</span>
                </div>
                <div>
                    <span className="text-large text-success">242</span><span className="text-small ">gb Avialable disk space</span>
                </div>
            </div>
        );
    }
}

export default HealthBlock;