import React from "react";
import Markdown from 'react-markdown';
import EditableText from "../util/EditableText";
import Requestor from "../util/Requestor";
import {LogbookLine} from "../statetypes";
import {toast} from "react-toastify";
import {ContextMenu, ContextMenuTrigger, MenuItem} from "react-contextmenu";

interface State {
    lines :LogbookLine[]
    input :string
    inputDisabled :boolean
    scrollStickToBottom :boolean
    dragSelectedRowid :number | null
    dragLastHoveredRowid :number | null
    dragCurrentHoverRowid :number | null
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
            dragSelectedRowid: null,
            dragLastHoveredRowid: null,
            dragCurrentHoverRowid: null,
        };
    }

    render() {
        let {input, inputDisabled} = this.state;
        let lines = this.lines();
        return <div className="block y-50 d-flex flex-column">
            <div className="overflow-auto flex-lg-grow-1" ref={(el) => this.scroller = el}
                 onScroll={this.handleUserScroll.bind(this)}>
                <table className="logtable" onMouseLeave={this.resetDragging.bind(this)}>
                    <tbody>
                    {this.renderDragIndicator()}
                    {this.state.dragSelectedRowid != null ?
                        <style>{".logtable .timestamp {user-select: none;}"}</style> : null}
                    {lines.map((line) =>
                        <LogbookLineComponent {...line}
                                              key={line.rowid}
                                              onMouseDown={this.lineOnMouseDown}
                                              onMouseUp={this.lineOnMouseUp}
                                              onMouseLeave={this.lineOnMouseLeave}
                                              onMouseEnter={this.lineOnMouseEnter}
                                              updateLine={this.updateLine}/>
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

    renderDragIndicator() {
        if (this.state.dragSelectedRowid == null || this.state.dragLastHoveredRowid == null || this.state.dragCurrentHoverRowid == null) {
            return null;
        }
        if (this.state.dragCurrentHoverRowid === this.state.dragSelectedRowid) {
            return null;
        }

        let originalLineIndex = this.lines().findIndex((line) => line.rowid === this.state.dragSelectedRowid);
        let currentLineIndex = this.lines().findIndex((line) => line.rowid === this.state.dragCurrentHoverRowid);
        let previousLineIndex = this.lines().findIndex((line) => line.rowid === this.state.dragLastHoveredRowid);
        let willBePlasedUnderCurrentLine = currentLineIndex > previousLineIndex;

        return <style>
            {`.logtable tbody tr:nth-of-type(${currentLineIndex + 1}) { border-${willBePlasedUnderCurrentLine ? "bottom" : "top"}: solid 3px #262f38;}
              .logtable tbody tr:nth-of-type(${originalLineIndex + 1}) {background-color: #262f38} 
              `}
        </style>
    }

    lineOnMouseDown = (rowid: number) => {
        if (this.state.dragSelectedRowid != null) {
            return;
        }
        this.setState({
            dragSelectedRowid: rowid,
            dragLastHoveredRowid: rowid
        })
    };

    lineOnMouseUp = () => {
        const {dragLastHoveredRowid, dragSelectedRowid, dragCurrentHoverRowid} = this.state;
        if (dragCurrentHoverRowid === dragSelectedRowid) {
            //did not move. Mouse released on same row as it was dragged from.
            this.resetDragging();
            return;
        }

        if (dragLastHoveredRowid == null || dragCurrentHoverRowid == null || dragSelectedRowid == null) {
            return
        }

        // The last line that was hoved over before the current line was hovered.
        let lastLineIndex = this.findIndexOfRow(dragLastHoveredRowid);

        // The line on which the mouse was released.
        let releasedLineIndex = this.findIndexOfRow(dragCurrentHoverRowid);

        let betweenAboveIndex, betweenBelowIndex;
        if (lastLineIndex < releasedLineIndex) {
            //moving down (positive)
            betweenAboveIndex = releasedLineIndex;
            betweenBelowIndex = releasedLineIndex + 1;
        } else if (lastLineIndex > releasedLineIndex) {
            //moving up (negative)
            betweenAboveIndex = releasedLineIndex - 1;
            betweenBelowIndex = releasedLineIndex;
        } else {
            // Cannot determen if we are moving up or down.
            // So we assume we did not move and do nothing.
            this.resetDragging();
            return
        }

        let loglineAboveTarget = this.lines()[betweenAboveIndex];
        let loglineBelowTarget = this.lines()[betweenBelowIndex];

        if ((loglineAboveTarget !== undefined && loglineAboveTarget.rowid === dragSelectedRowid) ||
            (loglineBelowTarget !== undefined && loglineBelowTarget.rowid === dragSelectedRowid)) {
            //did not move. Placed directly under or directly above current line.
            this.resetDragging();
            return;
        }

        let newTimestamp;
        if (loglineAboveTarget !== undefined && loglineBelowTarget !== undefined) {
            newTimestamp = Math.min(loglineAboveTarget.timestamp, loglineBelowTarget.timestamp) + Math.abs(loglineAboveTarget.timestamp - loglineBelowTarget.timestamp) / 2;
        } else if (loglineAboveTarget !== undefined) {
            newTimestamp = loglineAboveTarget.timestamp + 1
        } else if (loglineBelowTarget !== undefined) {
            newTimestamp = loglineBelowTarget.timestamp - 1;
        } else {
            this.resetDragging();
            return;
        }

        console.log(`Moving ${dragSelectedRowid} to timestamp ${newTimestamp}`);
        this.updateLine(dragSelectedRowid, {timestamp: newTimestamp});
        this.resetDragging();
    };

    lineOnMouseEnter = (rowid: number) => {
        if (this.state.dragSelectedRowid == null) {
            return
        }
        this.setState({
            dragCurrentHoverRowid: rowid
        })
    };

    lineOnMouseLeave = (rowid: number) => {
        if (this.state.dragSelectedRowid == null) {
            return
        }
        this.setState({
            dragLastHoveredRowid: rowid
        })
    };

    resetDragging() {
        this.setState({
            dragSelectedRowid: null,
            dragLastHoveredRowid: null,
        })
    }

    findIndexOfRow(rowid: number) {
        return this.lines().findIndex((line) => line.rowid === rowid)
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
    onMouseDown(rowid :number) :void
    onMouseUp(rowid :number) :void
    onMouseLeave(rowid :number) :void
    onMouseEnter(rowid :number) :void
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
        return <tr onMouseLeave={this.onMouseLeave.bind(this)}
                   onMouseEnter={this.onMouseEnter.bind(this)}
                   className={source !== "human" ? "text-small logbook-small" : ""}>
            <td className="timestamp text-grayyed"
                onMouseDown={this.onMouseDown.bind(this)}
                onMouseUp={this.onMouseUp.bind(this)}>
                {this.renderTimestamp(timestamp)}
            </td>
            <td>
                <EditableText value={text} save={(newText) => updateLine(rowid, {text: newText})} multiline={true}>
                    <Markdown source={text} className="pl-1 logline"/>
                </EditableText>
            </td>
            <td>
                <ContextMenuTrigger id={`service${this.props.rowid}logbooktrigger`} holdToDisplay={0}>
                    <div className={"text-small text-grayyed"}>⋮</div>
                </ContextMenuTrigger>
                <ContextMenu id={`service${this.props.rowid}logbooktrigger`}>
                    <MenuItem onClick={() => this.postToSlack()}>
                        Post to slack
                    </MenuItem>
                </ContextMenu>
            </td>
        </tr>
    }

    postToSlack() {
        const toastid = toast(`Posting message to slack...`, { autoClose: false });
        Requestor.slackLogbookLine(this.props.rowid)
            .then(() =>
                toast.update(toastid, {
                    render: "Posted message to slack",
                    type: toast.TYPE.SUCCESS,
                    autoClose: 5000
                })
            )
            .catch((error) => toast.update(toastid, {
                render: <span title={error}>Posting message to slack failed</span>,
                type: toast.TYPE.WARNING,
            }))
    }

    renderTimestamp(timestamp :number) {
        let date = new Date(timestamp * 1000);
        let hours = date.getHours();
        let minutes = "0" + date.getMinutes();
        let seconds = "0" + date.getSeconds();
        return hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2);
    }

    onMouseDown(e :React.MouseEvent<HTMLTableDataCellElement>) {
        if (e.button !== 0) return;
        this.props.onMouseDown(this.props.rowid);
    }

    onMouseUp(e :React.MouseEvent<HTMLTableDataCellElement>) {
        if (e.button !== 0) return;
        this.props.onMouseUp(this.props.rowid);
    }

    onMouseLeave(e :React.MouseEvent<HTMLTableRowElement>) {
        if (e.button !== 0) return;
        this.props.onMouseLeave(this.props.rowid);
    }

    onMouseEnter(e :React.MouseEvent<HTMLTableRowElement>) {
        if (e.button !== 0) return;
        this.props.onMouseEnter(this.props.rowid);
    }
}

export default LogbookBlock;

