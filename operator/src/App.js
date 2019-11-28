import React from 'react';
import io from 'socket.io-client';
import HealthBlock from './blocks/HealthBlock'
import NodesBlock from "./blocks/NodesBlock";
import TopicsBlock from "./blocks/TopicsBlock";
import ComputeboxBlock from "./blocks/ComputeboxBlock";
import ServicesBlock from "./blocks/ServicesBlock";
import GitBlock from "./blocks/GitBlock";
import LogbookBlock from "./blocks/LogbookBlock";
import RecordingBlock from "./blocks/RecordingBlock";


class App extends React.Component {

    constructor(props){
        super(props);
        this.state = {
            connectionerror: null
        };
    }

    render() {
        if(this.state.groundStationState === undefined) {
            return <div className="text-center">Groundstation offline</div>
        }
        return <div className="container-fluid">
            {this.renderDashboard()}
            {this.state.connectionerror === null ? null :
                <div className="overlay error text-center">{this.state.connectionerror}</div>}
        </div>

    }

    renderDashboard() {
        console.log(this.state.groundStationState);
        return <main id="page-main">
            <div className="row">
                <div className="col-xl-2 col-lg-4 col-sm-12 col-xs-12 gutter-small">
                    <HealthBlock/>
                </div>
                <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <NodesBlock/>
                </div>
                <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <TopicsBlock/>
                </div>
                <div className="col-xs-12 col-xl-6 gutter-small ">
                    <ComputeboxBlock groundStationState={this.state.groundStationState}/>
                    <ServicesBlock/>
                    <GitBlock/>
                </div>

            </div>
            <div className="row">
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <LogbookBlock groundStationState={this.state.groundStationState}/>
                </div>
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <RecordingBlock/>
                </div>
            </div>
        </main>
    }

    updateState() {
        // fetch("/state")
        //     .then((response) => {
        //         if (response.status !== 200 || !response.ok) {
        //             console.log("Connection to groundstation lost", response);
        //             this.setState({connectionerror: "Connection to groundstation lost: " + response.statusText})
        //         } else {
        //             response.json().then((data) => )
        //
        //         }
        //     })
    }

    componentDidMount() {
        this.socket = io({
            reconnectionDelayMax: 1000,
        });
        this.socket.on('state', (data) => {
            this.setState({
                groundStationState: data,
                connectionerror: null,
            })
        });
        this.socket.on('disconnect', () => {
            this.setState({
                connectionerror: "Disconnected from groundstation",
            })
        });

    }

    componentWillUnmount() {
        // clearInterval(this.interval);
    }
}

export default App;
