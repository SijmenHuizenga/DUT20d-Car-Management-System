import React from "react";
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor"
import {
    getTopicHealthPubs,
    Recording,
    Topic,
    TopicPublication,
} from "../statetypes";
import {Indicator, IndicatorColor} from "../util/Indicator";
import Tooltip from "../util/Tooltip";
import {toast} from "react-toastify";

interface Props extends Recording {
    topics :Topic[]
    publications :TopicPublication[]
}

class RecordingContainer extends React.Component<Props, {}> {
    render() {
        return this.props.is_recording ? <ActiveRecordingBlock {...this.props}/> : <InactiveRecordingBlock {...this.props} />
    }
}

class RecordingBlock extends React.Component<Props> {

    allTopicNames() :string[]{
        let out = [...this.props.config_topics, ...this.props.topics.map((topic) => topic.name)];
        out.sort(this.sortTopics);
        return out.filter((value, index, self) => self.indexOf(value) === index);
    }

    getTopicHealth(topicname :string) {
        return getTopicHealthPubs(
            this.props.topics.find((t) => t.name === topicname),
            this.props.publications.filter((p) => p.topicname === topicname),
        );
    }

    isTopicSelected(topicname :string) :boolean {
        return this.props.config_topics.indexOf(topicname) !== -1
    }

    onRecordingToggle() {
        const startstop = this.props.is_recording ? "stop" : "start";
        const toastid = toast(`${startstop} recording...`, { autoClose: false });

        return Requestor.put("/recording/" + startstop, {})
            .then(() =>
                toast.update(toastid, {
                    render: `${startstop} recording ok`,
                    type: toast.TYPE.SUCCESS,
                    autoClose: 5000
                })
            )
            .catch((error) => toast.update(toastid, {
                render: `Couldn't ${startstop} recording: ${error}`,
                type: toast.TYPE.ERROR
            }))
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

class InactiveRecordingBlock extends RecordingBlock {
    render() {
        let {config_filename, lastrefresh_config} = this.props;

        return <div className="block y-50 d-flex flex-fill flex-column">
            <div className="d-flex">
                <div className="text-large">
                    <Indicator color={IndicatorColor.idle} dataTimestamp={lastrefresh_config}/>
                    Idle
                </div>
                <div className="flex-grow-1 pl-2 d-flex align-items-center">
                    <EditableText value={config_filename} save={this.updateFilename.bind(this)} multiline={false} >
                        {config_filename}
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
            <div className="overflow-auto mt-1">
                {this.renderTopics()}
            </div>
        </div>
    }

    renderTopics() {
        if(!Array.isArray(this.props.config_topics)) {
            return "ERROR: " + this.props.config_topics
        }

        return this.allTopicNames().map((topicname) => {
            let [healthColor, healthDescription] = this.getTopicHealth(topicname);
            return <TopicSelector key={topicname}
                                  topicname={topicname}
                                  selected={this.isTopicSelected(topicname)}
                                  healthColor={healthColor}
                                  healthDescription={healthDescription}/>
        });
    }

    updateFilename(newname :string) {
        return Requestor.put("/recording/filename", {filename: newname})
            .then(() => {
                return true;
            })
            .catch((error) => {
                toast("Failed to update recording filename: " + error, {type: 'error'});
                return false;
            });
    }
}

class ActiveRecordingBlock extends RecordingBlock {
    render() {
        let {recording_file, recording_duration, recording_filesize, lastrefresh_recording} = this.props;
        return <div className="block y-50 d-flex flex-fill flex-column">
            <div className="mb-2 ">
                <span className="text-large"><Indicator color={IndicatorColor.active} dataTimestamp={lastrefresh_recording}/> Recording</span>
                <span className="pl-1 text-small">| {recording_file}</span>
                <span className="pl-1 text-small">| recording for {recording_duration}s</span>
                <span className="pl-1 text-small">| {recording_filesize}mb</span>
                <div className="float-right">
                    <button type="button"
                            className="btn btn-sm btn-outline-primary"
                            onClick={this.onRecordingToggle.bind(this)}>
                        Stop recording
                    </button>
                </div>
            </div>
            <div className="overflow-auto mt-1">
                {this.allTopicNames().map((topicname) => {
                    let [healthColor, healthDescription] = this.getTopicHealth(topicname);

                    return <TopicIndicator topicname={topicname}
                                           healthColor={healthColor}
                                           healthDescription={healthDescription}
                                           selected={this.isTopicSelected(topicname)}/>
                })}
            </div>
        </div>
    }
}

class TopicIndicator extends React.Component<{topicname :string, selected :boolean, healthColor :IndicatorColor, healthDescription :string}, {}> {
    render() {
        let {topicname, selected, healthColor, healthDescription} = this.props;
        return <div>
            <Tooltip tooltip={() => topicDescription(selected, healthDescription)}>
                <span className={`indicator circle ${selected ? healthColor : ""}`}/>
            </Tooltip>
            <span className="text-small pl-2">{topicname}</span>
        </div>
    }
}

class TopicSelector extends React.PureComponent<{topicname :string, selected :boolean, healthColor :IndicatorColor, healthDescription :string}, {}> {

    render() {
        let {topicname, selected, healthColor, healthDescription} = this.props;
        return <div className="custom-control custom-checkbox">
            <Tooltip tooltip={() => topicDescription(selected, healthDescription)}>
                <input type="checkbox"
                   className={`custom-control-input ${healthColor}`}
                   id={`recordtopic_${topicname}`}
                   checked={selected}
                   onChange={this.handleCheckboxChange.bind(this)}
                />
                <label
                    className={`custom-control-label ${healthColor}`}
                    htmlFor={`recordtopic_${topicname}`}>{topicname}</label>
            </Tooltip>
        </div>
    }

    handleCheckboxChange(e :React.ChangeEvent<HTMLInputElement>) {
        const toastid = toast(`${e.currentTarget.checked ? 'un' : ''}selecting topic ${this.props.topicname} for recording`,
            { autoClose: false });

        return Requestor.put("/recording/toggletopic", {
            topicname: this.props.topicname,
            selected: e.currentTarget.checked
        }).then(() => toast.update(toastid, {
            render: `${e.currentTarget.checked ? 'un' : ''}selected topic ${this.props.topicname} for recording`,
            type: toast.TYPE.SUCCESS,
            autoClose: 5000
        })).catch((error) => toast.update(toastid, {
            render: "Failed to update selected topcs: " + error,
            type: toast.TYPE.ERROR,
        }));
    }
}

function topicDescription(selected :boolean, health :string) {
    return `This topic is ${selected ? "" : "not"} selected for recording\n${health}`
}

export default RecordingContainer;

