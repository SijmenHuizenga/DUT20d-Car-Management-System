import React from "react";

class RecordingBlock extends React.Component {
    render() {
        return <div className="block">
            <div className="mb-2">
                <span className="text-large "><span className="indicator circle danger" /> Recording</span>
                <span className=" text-small">trackdrive_1_2019nov05.bag.active</span>
                <div className="float-right">
                    <button type="button" className="btn btn-sm btn-outline-primary">Stop recording
                    </button>
                </div>
            </div>
            <div className="d-flex flex-row mb-2">
                <select className="col form-control-sm mr-2">
                    <option>custom</option>
                    <option>Everything</option>
                    <option>Lidar</option>
                    <option>Lidar + Camera-raw</option>
                    <option>Lidar + Camera-raw + Maveros</option>
                </select>
                <button className="btn btn-sm btn-outline-primary mr-2">Load</button>
                <button className="btn btn-sm btn-outline-primary mr-2">Export</button>
                <button className="btn btn-sm btn-outline-primary">Save</button>
            </div>
            <div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/planning_BoundaryMarkers</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/planning_ReferencePath</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1" /><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/visualization_markers/world_state</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/visualization_markers/world_evidence</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label" htmlFor="customCheck1">/world_state</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/mavros/local_position/pose</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1/"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/mavros/local_position/velocity_local</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/mavros/local_position/velocity_body</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/lmpcc/prediction_space</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label" htmlFor="customCheck1">/world_evidence</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label" htmlFor="customCheck1">/mavros/imu/data_raw</label>
                </div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label"
                    htmlFor="customCheck1">/vehicle_interface/to/TrajectorySetpoints</label></div>
                <div className="custom-control custom-checkbox"><input type="checkbox"
                                                                       className="custom-control-input"
                                                                       id="customCheck1"/><label
                    className="custom-control-label" htmlFor="customCheck1">/tf</label></div>
            </div>
        </div>
    }
}
export default RecordingBlock;

