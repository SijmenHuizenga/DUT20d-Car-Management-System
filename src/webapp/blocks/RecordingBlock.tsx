import React from "react";
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor"
import {Recording, Topic} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";
import Tooltip from "../util/Tooltip";
import {toast} from "react-toastify";

interface Props extends Recording {
    topics :Topic[]
}

class RecordingContainer extends React.Component<Props, {}> {
    render() {
        return this.props.is_recording ? <ActiveRecordingBlock {...this.props}/> : <InactiveRecordingBlock {...this.props} />
    }
}

class RecordingBlock extends React.Component<Props> {

    allTopicNames() :string[]{
        let out = [...this.props.selected_topics, ...this.props.topics.map((topic) => topic.name)];
        out.sort();
        return out.filter((value, index, self) => self.indexOf(value) === index);
    }

    getTopicState(topicname :string) {
        // if (!this.props.topics.hasOwnProperty(topicname)) {
        //     return "OFFLINE";
        // }
        // return isRecent(this.props.topics[topicname].lastseen) ? "ACTIVE" : "OFFLINE"
        //todo: lastseen is not a good enough indicator to get state.
        return "OFFLINE"
    }

    isTopicSelected(topicname :string) :boolean {
        return this.props.selected_topics.indexOf(topicname) !== -1
    }

    onRecordingToggle() {
        return Requestor.put("/recording/" + (this.props.is_recording ? "stop" : "start"), {})
            .catch((error) => toast("Couldn't start/stop recording: " + error, {type: 'error'}))
    }

}

class InactiveRecordingBlock extends RecordingBlock {
    render() {
        let {filename} = this.props;

        return <div className="block">
            <div className="d-flex">
                <div className="text-large">
                    <Indicator color={IndicatorColor.idle} dataTimestamp={this.props.lastrefresh}/>
                    Idle
                </div>
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
                           selected={this.isTopicSelected(topicname)}
                           topicstate={this.getTopicState(topicname)}/>
        )
    }

    updateFilename(newname :string) {
        return Requestor.put("/recording/filename", {filename: newname})
            .then(() => {
                return true;
            })
            .catch((error) => {
                toast("Failed to update recording filename: " + error, {type: 'error'})
                return false;
            });
    }
}

class ActiveRecordingBlock extends RecordingBlock {
    render() {
        return <div className="block">
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
            <Tooltip tooltip={topicDescription(selected, topicstate)}>
                <span className={`indicator circle ${selected ? topicStateToColor(topicstate) : ""}`}/>
            </Tooltip>
            <span className="text-small pl-2">{topicname}</span>
        </div>
    }
}

class TopicSelector extends React.Component<{topicname :string, selected :boolean, topicstate :string}, {}> {
    render() {
        let {topicname, selected, topicstate} = this.props;
        return <div className="custom-control custom-checkbox">
            <Tooltip tooltip={topicDescription(selected, topicstate)}>
                <input type="checkbox"
                   className={`custom-control-input ${topicStateToColor(topicstate)}`}
                   id={`recordtopic_${topicname}`}
                   checked={selected}
                   onChange={this.handleCheckboxChange.bind(this)}
                />
                <label
                    className={`custom-control-label ${topicStateToColor(topicstate)}`}
                    htmlFor={`recordtopic_${topicname}`}>{topicname}</label>
            </Tooltip>
        </div>
    }

    handleCheckboxChange(e :React.ChangeEvent<HTMLInputElement>) {
        return Requestor.put("/recording/toggletopic", {
            topicname: this.props.topicname,
            selected: e.currentTarget.checked
        }).catch((error) => toast("Failed to update selected topcs: " + error, {type: 'error'}));
    }
}

function topicStateToColor(state :string) {
    //todo: enumify topic state
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
    return `This topic is ${selected ? "" : "not"} selected for recording\nThe topic is ${topicStateToDescription(topicstate)}`
}

export default RecordingContainer;

