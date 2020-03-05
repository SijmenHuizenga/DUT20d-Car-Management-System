function put(uri :string, body={}) {
    return execute(uri, "PUT", body)
}

function execute(uri :string, method="", body={}, timeout=5000): Promise<any> {
    let didTimeOut = false;
    return new Promise((resolve, reject) => {
        const timeoutTimer = setTimeout(()  => {
            didTimeOut = true;
            reject(new Error('Request timed out'));
        }, timeout);

        fetch(uri, {
            method: method,
            headers: method === 'GET' ? {} : {'Content-Type': 'application/json'},
            body: method === 'GET' ? null : JSON.stringify(body)
        }).then((response) => {
            clearTimeout(timeoutTimer);
            if(didTimeOut)
                return;
            if ((response.status === 201 || response.status === 200) && response.ok) {
                resolve(response);
            } else {
                console.log("request failed", response);
                response.text().then((body) => {
                    let errString = `Request failed with code ${response.status}: ${response.statusText}`;
                    if(body !== ""){
                        errString += '\n' + body;
                    }
                    reject(errString);
                });
            }
        })
    })
}



export default {
    runcommand(command :string) {
        return execute(`http://${window.location.hostname}:1098/runcommand`, "POST", {command})
    },
    setRecordingFilename(filename: string) {
        return put(`http://${window.location.hostname}:1099/recording/filename`, {filename})
    },
    setRecordingTopic(topicnames: string[], selected :boolean) {
        return put(`http://${window.location.hostname}:1099/recording/settopics`, {topicnames, selected})
    },
    getLogbook() {
        return execute(`http://${window.location.hostname}:1095/logbook`, 'GET')
    },
    addLogbookLine(text :string) {
        return execute(`http://${window.location.hostname}:1095/logbook`, 'POST', {text, source: 'human'})
    },
    updateLogbookLine(rowid :number, changeset :object) {
        return put(`http://${window.location.hostname}:1095/logbook/${rowid}`, changeset)
    },
    logbookSlackFullsync() {
        return execute(`http://${window.location.hostname}:1095/logbook/sync`, 'GET')
    }
}