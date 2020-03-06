import React, {ReactNode} from 'react';

var showModal: undefined | ((content :ReactNode | null) => void) = undefined;

export class ModalContainer extends React.Component<{}, {content: ReactNode | null}> {

    constructor(props: {}, context: any) {
        super(props, context);
        this.state = {
            content: null,
        };
        showModal = this.showModal;
    }

    render() {
        return this.state.content === null ? null
            : [<div className="modal show" tabIndex={-1} role="dialog" aria-hidden="false" style={{display: "block"}}
                    onClick={() => this.setState({content: null})}>
                <div className="modal-dialog modal-xl modal-dialog-scrollable" role="document"
                     onClick={(e) => e.stopPropagation()}>
                    <div className="modal-content">
                        {this.state.content}
                    </div>
                </div>
            </div>,
                <div className="modal-backdrop show"/>]
    }

    showModal = (content :ReactNode|null) => {
        this.setState({content})
    };

}

export default function (content :ReactNode|null) {
    if(showModal === undefined) {
        throw Error("showModal not defined")
    }
    showModal(content)
}