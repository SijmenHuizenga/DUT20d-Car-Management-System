import React from 'react';
import HealthBlock from './blocks/HealthBlock'
import NodesBlock from "./blocks/NodesBlock";
import TopicsBlock from "./blocks/TopicsBlock";
import ComputeboxBlock from "./blocks/ComputeboxBlock";
import ServicesBlock from "./blocks/ServicesBlock";
import GitBlock from "./blocks/GitBlock";
import LogbookBlock from "./blocks/LogbookBlock";
import RecordingBlock from "./blocks/RecordingBlock";

function App() {
  return (
      <div className="container-fluid">
          <main id="page-main">
              <div className="row">
                  <div className="col-xl-2 col-lg-4 col-sm-12 col-xs-12 gutter-small">
                      <HealthBlock />
                  </div>
                  <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                    <NodesBlock/>
                  </div>
                  <div className="col-xl-2 col-lg-4 col-sm-6 col-xs-12 gutter-small">
                      <TopicsBlock />
                  </div>
                  <div className="col-xs-12 col-xl-6 gutter-small ">
                      <ComputeboxBlock />
                      <ServicesBlock />
                      <GitBlock />
                  </div>

              </div>
              <div className="row">
                  <div className="col-xl-6 col-xs-12 gutter-small">
                      <LogbookBlock/>
                  </div>
                  <div className="col-xl-6 col-xs-12 gutter-small">
                      <RecordingBlock/>
                  </div>
              </div>
          </main>
      </div>
  );
}

export default App;
