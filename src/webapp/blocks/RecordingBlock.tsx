import React from "react";
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor"
import {lastPingWasRecent} from "../util/Timing";
import {Topic} from "../statetypes";

interface Props {
    selected_topics: string[]
    is_recording: boolean
    filename: string
    bagfilename :string
    recordingduration :string
    topics :{[key :string] :Topic}
}

interface State {
    error: string | null
}

class RecordingContainer extends React.Component<Props, {}> {
    render() {
        return this.props.is_recording ? <ActiveRecordingBlock {...this.props}/> : <InactiveRecordingBlock {...this.props} />
    }
}

class RecordingBlock extends React.Component<Props, State> {
    constructor(props :Props) {
        super(props);
        this.state = {
            error: null
        }
    }

    allTopicNames() :string[]{
        let out = [...this.props.selected_topics, ...Object.keys(this.props.topics)];
        out.sort();
        return out.filter((value, index, self) => self.indexOf(value) === index);
    }

    getTopicState(topicname :string) {
        if (!this.props.topics.hasOwnProperty(topicname)) {
            return "OFFLINE";
        }
        return lastPingWasRecent(this.props.topics[topicname].lastseen) ? "ACTIVE" : "OFFLINE"
    }

    isTopicSelected(topicname :string) :boolean {
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

    setError(error :string | null) {
        this.setState({
            error: error
        })
    }
}

class InactiveRecordingBlock extends RecordingBlock {
    render() {
        let {filename} = this.props;

        return <div className="block">
            {this.renderErrorblock()}
            <div className="d-flex">
                <div className="text-large">Idle</div>
                <div className="flex-grow-1 pl-1">
                    <EditableText value={filename} save={this.updateFilename.bind(this)} multiline={false} >
                        <span className="pl-1 text-small">{filename}</span>
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
                {this.renderTopics()}
            </div>
        </div>
    }

    renderTopics() {
        if(!Array.isArray(this.props.selected_topics)) {
            return "ERROR: " + this.props.selected_topics
        }
        return this.allTopicNames().map((topicname) =>
            <TopicSelector key={topicname}
                           topicname={topicname}
                           setError={this.setError.bind(this)}
                           selected={this.isTopicSelected(topicname)}
                           topicstate={this.getTopicState(topicname)}/>
        )
    }

    updateFilename(newname :string) {
        return Requestor.put("/recording/filename", {filename: newname})
            .then(() => {
                this.setError(null);
                return true;
            })
            .catch((error) => {
                this.setError("Failed to update recording filename: " + error);
                return false;
            });
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
                {this.allTopicNames().sort(this.sortTopics).map((topicname) =>
                        <TopicIndicator topicname={topicname}
                                        topicstate={this.getTopicState(topicname)}
                                        selected={this.isTopicSelected(topicname)}/>
                )}
            </div>
        </div>
    }

    sortTopics = (a :string, b :string) => {
        //put selected topics on top, sort each list on alphabetical order
        let selectedA = this.isTopicSelected(a);
        let selectedB = this.isTopicSelected(b);
        if (selectedA === selectedB) {
            return a.localeCompare(b);
        }
        return Number(selectedB) - Number(selectedA);
    };
}

class TopicIndicator extends React.Component<{topicname :string, selected :boolean, topicstate :string}, {}> {
    render() {
        let {topicname, selected, topicstate} = this.props;
        return <div>
            <span className={`indicator circle ${selected ? topicStateToColor(topicstate) : ""}`}
                  data-tip={topicDescription(selected, topicstate)}/>
            <span className="text-small pl-2">{topicname}</span>
        </div>
    }
}

class TopicSelector extends React.Component<{topicname :string, selected :boolean, topicstate :string, setError(msg :string|null) :void}, {}> {
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

    handleCheckboxChange(e :React.ChangeEvent<HTMLInputElement>) {
        return Requestor.put("/recording/toggletopic", {
            topicname: this.props.topicname,
            selected: e.currentTarget.checked
        })
            .then(() => this.props.setError(null))
            .catch((error) => this.props.setError("Failed to update selected topcs: " + error));
    }
}

function topicStateToColor(state :string) {
    switch (state) {
        case "OFFLINE":
            return "danger";
        case "IDLE":
            return "warning";
        case "ACTIVE":
            return "success";
        default: return "danger";
    }
}

function topicStateToDescription(state :string) {
    switch (state) {
        case "OFFLINE":
            return "Offline: it has no attached publishers or subscribers";
        case "IDLE":
            return "Idle: no messages are being sent recently";
        case "ACTIVE":
            return "Active: messages are being sent on this topic frequently";
        default: return `UNKOWN STATE ${state}`;
    }
}

function topicDescription(selected :boolean, topicstate :string) {
    return `This topic is ${selected ? "" : "not"} selected for recording<br/> The topic is ${topicStateToDescription(topicstate)}`
}

export default RecordingContainer;

