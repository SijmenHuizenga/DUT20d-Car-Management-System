function put(uri :string, body={}) {
    return execute(uri, "PUT", body)
}

function execute(uri :string, method="", body={}): Promise<any> {
    return new Promise((resolve, reject) => {
        fetch(uri, {
            method: method,
            headers: method === 'GET' ? {} : {'Content-Type': 'application/json'},
            body: method === 'GET' ? null : JSON.stringify(body)
        }).then((response) => {
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
    put, execute
}