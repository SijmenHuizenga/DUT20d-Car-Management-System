import React from "react";
import TextareaAutosize from "react-autosize-textarea";

interface Props {
    multiline: boolean
    value: string
    save(input: string): Promise<boolean>
}

interface State {
    editing: boolean
    input: string | null
    inputDisabled: boolean
}

class EditableText extends React.PureComponent<Props, State> {
    private inputref: React.RefObject<HTMLTextAreaElement>;

    constructor(props :Props) {
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
                    className="pl-1 d-flex"
                    value={this.state.input || ""}
                    onKeyDown={this.onKey.bind(this)}
                    onChange={(e :React.FormEvent<HTMLTextAreaElement>) => this.setState({input: e.currentTarget.value})}
                    disabled={this.state.inputDisabled}
                    ref={this.inputref}
                />
        }
        return <div onDoubleClick={this.startEditing.bind(this)}>
            {this.props.children}
        </div>;
    }

    onKey(e :React.KeyboardEvent) {
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
            this.props.save(this.state.input || "")
                .then((success :boolean) => {
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
        this.inputref.current!.focus()
    }

}

export default EditableText;
