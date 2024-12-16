// Flask로 명령 전송
function sendCommand(topic, command) {
    fetch("/send_command", {
        method: "POST",
        headers: {
            "Content-Type": "application/json",
        },
        body: JSON.stringify({ topic: topic, command: command }),
    })
    .then(response => response.json())
    .then(data => console.log(`Command Sent: ${command} to ${topic}`, data))
    .catch(err => console.error("Error sending command:", err));
}

// Case 명령 전송
function sendCaseCommand(caseType, value) {
    if (['a', 'b'].includes(caseType) && ([1, 2, 3].includes(value) || ['open', 'close'].includes(value))) {
        const command = `${caseType}:${value}`;
        sendCommand("case_command", command); // sendCommand 함수 호출
    } else {
        console.error("Invalid Case Command:", caseType, value);
    }
}
