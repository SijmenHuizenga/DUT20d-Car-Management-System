import React, {ReactNode} from 'react';

var showModal: undefined | ((content :ModalContent|null, title :string) => void) = undefined;

export type ModalContent = string | ReactNode

export class ModalContainer extends React.Component<{}, {content: ModalContent | null, title: string}> {

    constructor(props: {}, context: any) {
        super(props, context);
        this.state = {
            content: null,
            title: "",
        };
        showModal = this.showModal;
    }

    render() {
        return this.state.content === null ? null
            : <div className="modal fade show" tabIndex={-1} role="dialog" aria-hidden="false" style={{display: "block"}}>
                <div className="modal-dialog modal-lg" role="document">
                    <div className="modal-content">
                        <div className="modal-header">
                            <h5 className="modal-title">{this.state.title}</h5>
                            <button type="button" className="close" onClick={() => this.setState({content: null})}>
                                &times;
                            </button>
                        </div>
                        <div className="modal-body">
                            {this.state.content}
                        </div>
                    </div>
                </div>
            </div>
    }

    showModal = (content :ModalContent, title :string) => {
        this.setState({content, title})
    };

}

export default function (content :ModalContent|null, title :string) {
    if(showModal !== undefined) {
        showModal(content, title)
    } else {
        throw Error("showModal not defined")
    }
}