import React from "react";
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor"

class RecordingContainer extends React.Component {
    render() {
        return this.props.is_recording ? <ActiveRecordingBlock {...this.props}/> : <InactiveRecordingBlock {...this.props} />
    }
}

class RecordingBlock extends React.Component {
    constructor(props) {
        super(props)
        this.state = {
            error: null
        }
    }

    allTopics() {
        let out = [...this.props.selected_topics, ...Object.keys(this.props.topics)];
        out.sort();
        return out.filter((value, index, self) => self.indexOf(value) === index);
    }

    getTopicState(topicname) {
        if (this.props.topics.hasOwnProperty(topicname)) {
            return this.props.topics[topicname].state;
        }
        return "OFFLINE";
    }

    isTopicSelected(topicname) {
        return this.props.selected_topics.indexOf(topicname) !== -1
    }

    onRecordingToggle() {
        return Requestor.put("/recording/" + (this.props.is_recording ? "stop" : "start"), {})
            .then(() => this.setError(null))
            .catch((error) => this.setError("Couldn't start/stop recording: " + error))
    }

    renderErrorblock() {
        return this.state.error ? <div className="alert alert-danger" role="alert">{this.state.error}</div> : null;
    }

    setError(error) {
        this.setState({
            error: error
        })
    }
}

class InactiveRecordingBlock extends RecordingBlock {
    render() {
        let {bagname} = this.props;

        return <div className="block">
            {this.renderErrorblock()}
            <div className="d-flex">
                <div className="text-large">Idle</div>
                <div className="flex-grow-1 pl-1">
                    <EditableText value={bagname} save={this.updateFilename.bind(this)} multiline={false} >
                        <span className="pl-1 text-small">{bagname}</span>
                    </EditableText>
                </div>
                <div>
                    <button type="button"
                            className="btn btn-sm btn-outline-primary"
                            onClick={this.onRecordingToggle.bind(this)}>
                        Start recording
                    </button>
                </div>
            </div>
            <div>
                {this.allTopics().map((topicname) =>
                    <TopicSelector key={topicname}
                                   topicname={topicname}
                                   setError={this.setError.bind(this)}
                                   selected={this.isTopicSelected(topicname)}
                                   topicstate={this.getTopicState(topicname)}/>
                )}
            </div>
        </div>
    }
    updateFilename(newname) {
        return Requestor.put("/recording/", {filename: newname})
            .then(() => this.setError(null))
            .catch((error) => this.setError("Failed to update recording filename: " + error));
    }
}

class ActiveRecordingBlock extends RecordingBlock {
    render() {
        return <div className="block">
            {this.renderErrorblock()}
            <div className="mb-2 ">
                <span className="text-large">Recording</span>
                <span className="pl-1 text-small">| {this.props.bagfilename}</span>
                <span className="pl-1 text-small">| recording for {this.props.recordingduration}</span>
                <div className="float-right">
                    <button type="button"
                            className="btn btn-sm btn-outline-primary"
                            onClick={this.onRecordingToggle.bind(this)}>
                        Stop recording
                    </button>
                </div>
            </div>
            <div>
                {this.allTopics().sort(this.sortTopics).map((topicname) =>
                        <TopicIndicator topicname={topicname}
                                        topicstate={this.getTopicState(topicname)}
                                        selected={this.isTopicSelected(topicname)}/>
                )}
            </div>
        </div>
    }

    sortTopics = (a, b) => {
        //put selected topics on top, sort each list on alphabetical order
        let selectedA = this.isTopicSelected(a);
        let selectedB = this.isTopicSelected(b);
        if (selectedA === selectedB) {
            return a - b;
        }
        return selectedB - selectedA;
    };
}

class TopicIndicator extends React.Component {
    render() {
        let {topicname, selected, topicstate} = this.props;
        return <div>
            <span className={`indicator circle ${selected ? topicStateToColor(topicstate) : ""}`}
                  data-tip={topicDescription(selected, topicstate)}/>
            <span className="text-small pl-2">{topicname}</span>
        </div>
    }
}

class TopicSelector extends React.Component {
    render() {
        let {topicname, selected, topicstate} = this.props;
        return <div className="custom-control custom-checkbox">
            <input type="checkbox"
                   className={`custom-control-input ${topicStateToColor(topicstate)}`}
                   id={`recordtopic_${topicname}`}
                   checked={selected}
                   onChange={this.handleCheckboxChange.bind(this)}
            />
            <label
                className="custom-control-label"
                data-tip={topicDescription(selected, topicstate)}
                htmlFor={`recordtopic_${topicname}`}>{topicname}</label>
        </div>
    }

    handleCheckboxChange(e) {
        return Requestor.put("/recording/toggletopic", {
            topicname: this.props.topicname,
            selected: e.target.checked
        })
            .then(() => this.props.setError(null))
            .catch((error) => this.props.setError("Failed to update recording filename: " + error));
    }
}

function topicStateToColor(state) {
    switch (state) {
        case "OFFLINE":
            return "danger";
        case "IDLE":
            return "warning";
        case "ACTIVE":
            return "success";
    }
}

function topicStateToDescription(state) {
    switch (state) {
        case "OFFLINE":
            return "Offline: it has no attached publishers or subscribers";
        case "IDLE":
            return "Idle: no messages are being sent recently";
        case "ACTIVE":
            return "Active: messages are being sent on this topic frequently";
    }
}

function topicDescription(selected, topicstate) {
    return `This topic is ${selected ? "" : "not"} selected for recording<br/> The topic is ${topicStateToDescription(topicstate)}`
}

export default RecordingContainer;

