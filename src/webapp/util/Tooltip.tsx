import React from 'react';

interface Props {
    tooltip: string | JSX.Element
}

class Tooltip extends React.Component<Props, {}> {

    render() {
        return (
            <div className="tooltp">
                {this.props.children}
                <div className="tooltptext">
                    { typeof this.props.tooltip == "string" ?
                        this.props.tooltip.split('\n').map((item, key) => {
                            return <React.Fragment key={key}>{item}<br/></React.Fragment>
                        }) : this.props.tooltip
                    }
                </div>
            </div>
        )
    }
}

export default Tooltip