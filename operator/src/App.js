import React from 'react';
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
        this.state = {};
    }

    render() {
        return <div className="container-fluid">
            {this.state.groundStationState === undefined
                ? "Did not yet receive data form ground station"
                : this.renderDashboard()}
        </div>
    }

    renderDashboard() {
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
                    <LogbookBlock/>
                </div>
                <div className="col-xl-6 col-xs-12 gutter-small">
                    <RecordingBlock/>
                </div>
            </div>
        </main>
    }

    updateState() {
        fetch("/state")
            .then((response) => response.json())
            .then((newState) => this.setState({groundStationState: newState}))
    }

    componentDidMount() {
        this.updateState();
        this.interval = setInterval(this.updateState.bind(this), 1000);
    }

    componentWillUnmount() {
        clearInterval(this.interval);
    }
}

export default App;
