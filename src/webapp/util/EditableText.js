import React from "react";
import TextareaAutosize from "react-autosize-textarea";

class EditableText extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            editing: false,
            input: null,
            inputDisabled: false,
        };
        this.inputref = React.createRef()
    }

    render() {
        if (this.state.editing) {
            return <TextareaAutosize
                    maxRows={this.props.multiline ? 10 : 1}
                    className="pl-1"
                    value={this.state.input}
                    onKeyDown={this.onKey.bind(this)}
                    onChange={(e) => this.setState({input: e.target.value})}
                    disabled={this.state.inputDisabled}
                    ref={this.inputref}
                />
        }
        return <div onDoubleClick={this.startEditing.bind(this)} style={{display: "inline-block"}}>
            {this.props.children}
        </div>;
    }

    onKey(e) {
        if (e.key === "Escape") {
            this.stopEditing();
            return;
        }
        if (e.key === "Enter") {
            if(this.props.multiline && e.shiftKey) {
                return;
            }
            this.saveEditing();
        }
    }

    startEditing() {
        this.setState({
            editing: true,
            input: this.props.value
        }, () => this.focusEditor());
    }

    stopEditing() {
        this.setState({
            editing: false,
            input: null
        })
    }

    saveEditing() {
        if (this.state.input === "") {
            alert("Cannot have empty message");
            return
        }
        this.setState({
            inputDisabled: true,
        }, () => {
            this.props.save(this.state.input)
                .then((success) => {
                    if (success) {
                        this.setState({
                            editing: false,
                            input: null,
                            inputDisabled: false
                        })
                    } else {
                        this.setState({
                            inputDisabled: false
                        }, () => this.focusEditor())
                    }
                })
        })
    }

    focusEditor() {
        this.inputref.current.focus();
    }

}

export default EditableText;
