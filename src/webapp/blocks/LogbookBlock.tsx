import React from "react";
import Markdown from 'react-markdown';
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor";
import {LogbookLine} from "../statetypes";
import {toast} from "react-toastify";

interface State {
    lines :LogbookLine[]
    input :string
    inputDisabled :boolean
    scrollStickToBottom :boolean
    fullsyncInProgress :boolean
}

class LogbookBlock extends React.Component<{}, State> {
    private scroller: HTMLDivElement | null;
    private inputfield: HTMLInputElement | null;
    private timerID: NodeJS.Timeout | null;

    constructor(props: {}) {
        super(props);
        this.inputfield = null;
        this.scroller = null;
        this.timerID = null;
        this.state = {
            lines: [],
            input: '',
            inputDisabled: false,
            scrollStickToBottom: true,
            fullsyncInProgress: false
        };
    }

    render() {
        let {input, inputDisabled} = this.state;
        let lines = this.lines();

        return <div className="block y-50 d-flex flex-column">
            {this.renderSyncBtn(lines)}

            <div className="overflow-auto flex-lg-grow-1" ref={(el) => this.scroller = el}
                 onScroll={this.handleUserScroll.bind(this)}>

                <table className="logtable">
                    <tbody>
                    {lines.map((line) =>
                        <LogbookLineComponent {...line} key={line.rowid} updateLine={this.updateLine}/>
                    )}
                    </tbody>
                </table>
            </div>
            <div className="input-group pt-1 input-group-sm">
                <input type="text" className="form-control" value={input}
                       onChange={(e) => this.setState({input: e.target.value})}
                       onKeyDown={this.handleKeyDown.bind(this)}
                       disabled={inputDisabled}
                       ref={(el) => this.inputfield = el}/>
            </div>
        </div>
    }

    private renderSyncBtn(lines :LogbookLine[]) {
        if(this.state.fullsyncInProgress) {
            return <span className={"btn btn-link btn-sm"}>...</span>
        }

        let slackOtOfSyncLies = 0;
        for(let i = 0; i < lines.length; ++i){
            if(lines[i].lastupdated !== lines[i].slacklastupdated)
                slackOtOfSyncLies++
        }

        if(slackOtOfSyncLies === 0) {
            return null
        }
        return <span onClick={this.slackfullsync.bind(this)} className={"btn btn-link btn-sm"}>Slack out of sync. Click here to update {slackOtOfSyncLies} lines to slack</span>
    }

    slackfullsync() {
        this.setState({fullsyncInProgress: true});
        Requestor.logbookSlackFullsync()
            .then((response) => response.json())
            .then((json) => {
                console.log("json", json);
                this.setState({fullsyncInProgress: false, lines: json.logbook});
                if(json.syncfailedlines > 0) {
                    toast(json.syncfailedlines + ' lines could not be synced to slack. Check the network connection of the cms laptop.', {type: 'error'});
                }
            })
            .catch(() => {
                toast('Full slack sync request failed.', {type: 'error'});
                this.setState({fullsyncInProgress: false})
            });
    }

    handleUserScroll() {
        let n = this.isScrolledToBottom();
        if (n !== this.state.scrollStickToBottom)
            this.setState({scrollStickToBottom: n});
    }

    componentDidMount() {
        this.scrollToBottom();
        this.timerID = setInterval(
            () => this.reloadLogbook(),
            1000
        );
    }

    componentWillUnmount() {
        clearInterval(this.timerID!);
    }

    componentDidUpdate() {
        this.scrollToBottom();
    }

    isScrolledToBottom() {
        return this.scroller!.scrollTop === (this.scroller!.scrollHeight - this.scroller!.offsetHeight)
    }

    scrollToBottom() {
        if (this.state.scrollStickToBottom)
            this.scroller!.scrollTop = this.scroller!.scrollHeight - this.scroller!.offsetHeight;
    }

    handleKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
        if (e.key === 'Enter') {
            this.storeNewLine()
        }
        return false;
    }

    reloadLogbook() {
        Requestor.getLogbook()
            .then((response) => response.json())
            .then((json) => this.setState({lines: json}))
            .catch((error) => {
                //todo show some kind of error
            })
    }

    storeNewLine() {
        this.setState({inputDisabled: true, scrollStickToBottom: true});

        Requestor.addLogbookLine(this.state.input)
            .then((response) => response.json())
            .then((json) => this.setState({inputDisabled: false, input: '', lines: json}))
            .catch((error) => {
                toast('Failed to store logline: ' + error, {type: 'error'});
                this.setState({
                    inputDisabled: false,
                })
            });
    }

    updateLine = (rowid: number, changeset: any) => {
        return Requestor.updateLogbookLine(rowid, changeset)
            .then((response) => response.json())
            .then((json) => this.setState({lines: json}))
            .then(() => true)
            .catch((error) => {
                toast('Failed to update logline: ' + error, {type: 'error'});
                return false;
            });
    };

    lines = () => [...this.state.lines].sort((a, b) => a.timestamp - b.timestamp);

}

interface LineProps extends LogbookLine {
    updateLine(rowid :number, changeset :any) :Promise<boolean>
}

class LogbookLineComponent extends React.PureComponent<LineProps, {moving :boolean}> {
    constructor(props :LineProps) {
        super(props);
        this.state = {
            moving: false
        };
    }

    render() {
        let {rowid, timestamp, text, updateLine, source} = this.props;
        return <tr className={source !== "human" ? "text-small logbook-small" : ""}>
            <td className="timestamp text-grayyed">
                {this.renderTimestamp(timestamp)}
            </td>
            <td>
                <EditableText value={text} save={(newText) => updateLine(rowid, {text: newText})} multiline={true}>
                    <Markdown source={text} className="pl-1 logline"/>
                </EditableText>
            </td>
        </tr>
    }

    renderTimestamp(timestamp :number) {
        let date = new Date(timestamp * 1000);
        let hours = date.getHours();
        let minutes = "0" + date.getMinutes();
        let seconds = "0" + date.getSeconds();
        return hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2);
    }

}

export default LogbookBlock;

