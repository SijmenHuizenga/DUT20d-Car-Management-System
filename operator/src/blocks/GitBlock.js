import React from "react";

class GitBlock extends React.Component {
    render() {
        return <div className="block">
                <div>
                                      <span className="text-large "><span
                                          className="indicator circle danger" /> master &nbsp;</span>
                    <div className="float-right">
                        <button type="button" className="btn btn-sm btn-outline-primary">Pull</button>
                        <button type="button" className="btn btn-sm btn-outline-primary">Fetch
                        </button>
                        <button type="button" className="btn btn-sm btn-outline-primary">Catkin
                            build
                        </button>
                        <button type="button" className="btn btn-sm btn-outline-danger ">Delete local
                            changes
                        </button>
                    </div>
                </div>
                <div className="mb-2 text-small" style={{lineHeight: "15px"}}>Added boost button to manualjoy stickco ntrded boost button to ma</div>
                <div className="d-flex flex-row mb-2">
                    <select className="col form-control-sm mr-2">
                        <option>master</option>
                    </select>
                    <button className="col-xs-1 btn btn-sm btn-outline-primary">Checkout</button>
                </div>
            </div>
    }
}

export default GitBlock;

